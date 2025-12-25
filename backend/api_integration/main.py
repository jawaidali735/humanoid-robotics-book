"""
Main FastAPI application for the Humanoid Robotics Chat API.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
import uvicorn
from .config import settings
from .middleware import add_logging_middleware, add_error_handlers

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Create FastAPI app instance with proper configuration
app = FastAPI(
    title=settings.API_TITLE,
    description=settings.API_DESCRIPTION,
    version=settings.API_VERSION,
    debug=settings.DEBUG,
)

# Add rate limit exception handler
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Configure CORS middleware for Docusaurus frontend origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Import and include routers after app creation to avoid circular imports
from .routers import chat

# Include the chat router
app.include_router(chat.chat_router)

# Add logging middleware
add_logging_middleware(app)

# Add error handlers
add_error_handlers(app)

@app.get("/")
async def root():
    """Root endpoint for basic health check."""
    return {
        "message": "Humanoid Robotics Chat API",
        "version": settings.API_VERSION,
        "status": "running"
    }

if __name__ == "__main__":
    # Run the application with uvicorn
    uvicorn.run(
        "api_integration.main:app",
        host=settings.HOST,
        port=settings.PORT,
        reload=settings.DEBUG,
    )