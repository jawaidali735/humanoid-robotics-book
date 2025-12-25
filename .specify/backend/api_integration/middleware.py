"""
Middleware for the Humanoid Robotics Chat API.
"""
import time
import logging
from typing import Callable, Awaitable
from fastapi import Request, Response
from fastapi.responses import JSONResponse
from .config import settings
from .monitoring import metrics_collector, RequestMetrics
from datetime import datetime
from uuid import uuid4

# Set up logging
logging.basicConfig(level=logging.INFO if not settings.DEBUG else logging.DEBUG)
logger = logging.getLogger(__name__)

class LoggingMiddleware:
    """Middleware for request/response logging with performance monitoring."""

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send) -> None:
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        request = Request(scope)
        start_time = time.time()
        request_id = str(uuid4())

        # Capture the original send function to intercept the response
        response_body = []
        original_send = send
        status_code = None

        async def capture_response_message(message):
            nonlocal status_code
            if message["type"] == "http.response.start":
                status_code = message["status"]
            elif message["type"] == "http.response.body":
                response_body.append(message.get("body", b""))
            await original_send(message)

        # Process the request
        await self.app(scope, receive, capture_response_message)

        # Calculate processing time
        process_time = time.time() - start_time

        # Record performance metrics
        client_ip = self.get_client_ip(request)
        metric = RequestMetrics(
            request_id=request_id,
            endpoint=request.url.path,
            method=request.method,
            response_time=process_time,
            status_code=status_code or scope.get('status', 200),
            timestamp=datetime.utcnow(),
            client_ip=client_ip,
            user_agent=request.headers.get("user-agent", "unknown")
        )
        metrics_collector.record_request(metric)

        # For concurrent user tracking, we'll log the activity but not track individual connections
        # since HTTP requests are stateless. For true concurrent user tracking, we'd need
        # WebSocket connections or session-based tracking.
        # Instead, we'll just ensure the metrics collector is aware of this request
        # and provide a method to calculate approximate concurrent users based on
        # requests in a given time window.

        # Log the request
        logger.info(
            f"Request: {request.method} {request.url.path} "
            f"Status: {status_code or scope.get('status', 'unknown')} "
            f"Process Time: {process_time:.2f}s "
            f"IP: {self.get_client_ip(request)} "
            f"Request ID: {request_id}"
        )

        # Log performance warnings for slow requests
        if process_time > 5.0:  # More than 5 seconds
            logger.warning(
                f"Slow API call detected: {request.method} {request.url.path} took {process_time:.2f}s",
                extra={'request_id': request_id, 'response_time': process_time}
            )

    def get_client_ip(self, request: Request) -> str:
        """Get client IP address from request."""
        forwarded_for = request.headers.get("x-forwarded-for")
        if forwarded_for:
            return forwarded_for.split(",")[0]
        forwarded_host = request.headers.get("x-forwarded-host")
        real_ip = request.headers.get("x-real-ip")
        x_cluster_client_ip = request.headers.get("x-cluster-client-ip")

        if forwarded_for:
            return forwarded_for.split(",")[0].strip()
        elif forwarded_host:
            return forwarded_host
        elif real_ip:
            return real_ip
        elif x_cluster_client_ip:
            return x_cluster_client_ip.split(",")[0].strip()
        else:
            return request.client.host

def add_logging_middleware(app):
    """Add logging middleware to the FastAPI app."""
    app.add_middleware(LoggingMiddleware)

# Error handling middleware for consistent error responses
async def custom_http_exception_handler(request: Request, exc: Exception):
    """Custom HTTP exception handler for consistent error responses."""
    logger.error(f"Error processing request: {exc}")

    error_response = {
        "success": False,
        "data": {},
        "error": {
            "code": "INTERNAL_ERROR",
            "message": "An internal error occurred while processing the request",
            "details": str(exc) if settings.DEBUG else None
        }
    }

    return JSONResponse(
        status_code=500,
        content=error_response
    )

def add_error_handlers(app):
    """Add error handlers to the FastAPI app."""
    app.add_exception_handler(Exception, custom_http_exception_handler)