"""
Performance monitoring and metrics for the Humanoid Robotics Chat API
"""
import time
import logging
from typing import Dict, Any, Optional
from datetime import datetime, timedelta
from dataclasses import dataclass, field
from collections import defaultdict
import threading

logger = logging.getLogger(__name__)

@dataclass
class RequestMetrics:
    """Data class to store request metrics."""
    request_id: str
    endpoint: str
    method: str
    response_time: float
    status_code: int
    timestamp: datetime
    user_agent: Optional[str] = None
    client_ip: Optional[str] = None
    query_length: Optional[int] = None

class MetricsCollector:
    """Collects and manages API performance metrics."""

    def __init__(self):
        self._metrics: Dict[str, RequestMetrics] = {}
        self._endpoint_stats: Dict[str, Dict[str, Any]] = defaultdict(lambda: {
            'count': 0,
            'total_time': 0.0,
            'avg_time': 0.0,
            'min_time': float('inf'),
            'max_time': 0.0,
            'error_count': 0
        })
        self._active_connections: Dict[str, datetime] = {}  # Track active connections by IP
        self._lock = threading.Lock()

    def record_request(self, metrics: RequestMetrics):
        """Record a request metric."""
        with self._lock:
            self._metrics[metrics.request_id] = metrics

            # Update endpoint statistics
            endpoint_key = f"{metrics.method} {metrics.endpoint}"
            stats = self._endpoint_stats[endpoint_key]

            stats['count'] += 1
            stats['total_time'] += metrics.response_time
            stats['avg_time'] = stats['total_time'] / stats['count']
            stats['min_time'] = min(stats['min_time'], metrics.response_time)
            stats['max_time'] = max(stats['max_time'], metrics.response_time)

            if metrics.status_code >= 400:
                stats['error_count'] += 1

    def get_endpoint_stats(self, endpoint: str = None) -> Dict[str, Any]:
        """Get statistics for one or all endpoints."""
        with self._lock:
            if endpoint:
                # Find stats for specific endpoint
                for key, stats in self._endpoint_stats.items():
                    if endpoint in key:
                        return {key: stats}
                return {}
            else:
                return dict(self._endpoint_stats)

    def get_overall_stats(self) -> Dict[str, Any]:
        """Get overall API statistics."""
        with self._lock:
            total_requests = sum(stats['count'] for stats in self._endpoint_stats.values())
            total_errors = sum(stats['error_count'] for stats in self._endpoint_stats.values())

            if total_requests > 0:
                avg_response_time = sum(stats['total_time'] for stats in self._endpoint_stats.values()) / total_requests
                error_rate = (total_errors / total_requests) * 100
            else:
                avg_response_time = 0.0
                error_rate = 0.0

            return {
                'total_requests': total_requests,
                'total_errors': total_errors,
                'error_rate_percent': error_rate,
                'average_response_time': avg_response_time,
                'timestamp': datetime.utcnow().isoformat()
            }

    def get_recent_metrics(self, limit: int = 100) -> Dict[str, RequestMetrics]:
        """Get recent request metrics."""
        with self._lock:
            # Sort by timestamp and return the most recent
            sorted_metrics = sorted(
                self._metrics.values(),
                key=lambda m: m.timestamp,
                reverse=True
            )
            return {m.request_id: m for m in sorted_metrics[:limit]}

    def track_connection_start(self, client_ip: str):
        """Track when a connection starts."""
        with self._lock:
            self._active_connections[client_ip] = datetime.utcnow()

    def track_connection_end(self, client_ip: str):
        """Track when a connection ends."""
        with self._lock:
            if client_ip in self._active_connections:
                del self._active_connections[client_ip]

    def get_active_connections(self) -> Dict[str, datetime]:
        """Get currently active connections."""
        with self._lock:
            return self._active_connections.copy()

    def get_concurrent_user_count(self) -> int:
        """Get the number of currently active connections."""
        with self._lock:
            return len(self._active_connections)

    def get_approximate_concurrent_users(self, time_window_minutes: int = 5) -> int:
        """
        Get approximate number of concurrent users based on requests in the time window.
        This is an estimation based on requests that occurred within the specified time window.
        """
        with self._lock:
            current_time = datetime.utcnow()
            time_threshold = current_time - timedelta(minutes=time_window_minutes)

            # Count unique IPs that made requests within the time window
            recent_ips = set()
            for metric in self._metrics.values():
                if metric.timestamp >= time_threshold:
                    if metric.client_ip:
                        recent_ips.add(metric.client_ip)

            return len(recent_ips)

    def cleanup_old_connections(self, max_age_minutes: int = 30):
        """Remove connections that have been active for longer than max_age_minutes."""
        with self._lock:
            current_time = datetime.utcnow()
            old_connections = []
            for ip, timestamp in self._active_connections.items():
                if (current_time - timestamp).total_seconds() > max_age_minutes * 60:
                    old_connections.append(ip)

            for ip in old_connections:
                del self._active_connections[ip]

            return len(old_connections)

# Global metrics collector instance
metrics_collector = MetricsCollector()

def track_performance(endpoint: str, method: str):
    """Decorator to track performance of API endpoints."""
    def decorator(func):
        async def wrapper(*args, **kwargs):
            start_time = time.time()
            request = kwargs.get('request') or (args[0] if args else None)

            try:
                response = await func(*args, **kwargs)
                status_code = 200  # Default success
                # Note: In a real implementation, we'd need to get the actual status code
            except Exception as e:
                status_code = 500
                raise
            finally:
                response_time = time.time() - start_time

                # Extract additional info if available
                client_ip = getattr(request, 'client', {}).get('host', 'unknown') if request else 'unknown'
                user_agent = 'unknown'  # Would extract from request headers in real implementation

                # Record the metric
                from uuid import uuid4
                metric = RequestMetrics(
                    request_id=str(uuid4()),
                    endpoint=endpoint,
                    method=method,
                    response_time=response_time,
                    status_code=status_code,
                    timestamp=datetime.utcnow(),
                    client_ip=client_ip,
                    user_agent=user_agent
                )

                metrics_collector.record_request(metric)

                # Log performance if response time is above threshold
                if response_time > 5.0:  # 5 seconds
                    logger.warning(
                        f"Slow API call: {method} {endpoint} took {response_time:.2f}s",
                        extra={'response_time': response_time}
                    )

            return response
        return wrapper
    return decorator

def get_performance_metrics():
    """Get current performance metrics."""
    return {
        'overall_stats': metrics_collector.get_overall_stats(),
        'endpoint_stats': metrics_collector.get_endpoint_stats(),
        'timestamp': datetime.utcnow().isoformat()
    }


def get_usage_analytics(time_window_hours: int = 24) -> Dict[str, Any]:
    """
    Get comprehensive API usage analytics for the specified time window.

    Args:
        time_window_hours: Number of hours to look back for analytics

    Returns:
        Dictionary containing usage analytics
    """
    current_time = datetime.utcnow()
    time_threshold = current_time - timedelta(hours=time_window_hours)

    with metrics_collector._lock:
        # Filter metrics within the time window
        filtered_metrics = [
            metric for metric in metrics_collector._metrics.values()
            if metric.timestamp >= time_threshold
        ]

        if not filtered_metrics:
            return {
                'time_window_hours': time_window_hours,
                'total_requests': 0,
                'unique_users': 0,
                'endpoints_usage': {},
                'error_rate': 0.0,
                'avg_response_time': 0.0,
                'most_active_hours': [],
                'peak_usage_times': [],
                'timestamp': current_time.isoformat()
            }

        # Calculate various analytics
        total_requests = len(filtered_metrics)

        # Count unique users (by IP)
        unique_users = len(set(metric.client_ip for metric in filtered_metrics if metric.client_ip))

        # Count requests by endpoint
        endpoint_counts = defaultdict(int)
        for metric in filtered_metrics:
            endpoint_key = f"{metric.method} {metric.endpoint}"
            endpoint_counts[endpoint_key] += 1

        # Calculate error rate
        error_count = sum(1 for metric in filtered_metrics if metric.status_code >= 400)
        error_rate = (error_count / total_requests) * 100 if total_requests > 0 else 0.0

        # Calculate average response time
        response_times = [metric.response_time for metric in filtered_metrics]
        avg_response_time = sum(response_times) / len(response_times) if response_times else 0.0

        # Find most active hours (by hour of day)
        hourly_activity = defaultdict(int)
        for metric in filtered_metrics:
            hour = metric.timestamp.hour
            hourly_activity[hour] += 1

        most_active_hours = sorted(hourly_activity.items(), key=lambda x: x[1], reverse=True)[:5]

        # Find peak usage times (by specific timestamps)
        peak_usage_times = sorted(
            [(metric.timestamp, metric.response_time) for metric in filtered_metrics],
            key=lambda x: x[1], reverse=True
        )[:10]

        return {
            'time_window_hours': time_window_hours,
            'total_requests': total_requests,
            'unique_users': unique_users,
            'endpoints_usage': dict(endpoint_counts),
            'error_rate': error_rate,
            'avg_response_time': avg_response_time,
            'most_active_hours': most_active_hours,
            'peak_usage_times': [(t.isoformat(), rt) for t, rt in peak_usage_times],
            'timestamp': current_time.isoformat()
        }


def get_user_analytics(client_ip: str = None) -> Dict[str, Any]:
    """
    Get analytics for a specific user or overall user patterns.

    Args:
        client_ip: Specific client IP to analyze, or None for overall patterns

    Returns:
        Dictionary containing user analytics
    """
    with metrics_collector._lock:
        if client_ip:
            user_metrics = [
                metric for metric in metrics_collector._metrics.values()
                if metric.client_ip == client_ip
            ]
        else:
            user_metrics = list(metrics_collector._metrics.values())

        if not user_metrics:
            return {
                'client_ip': client_ip,
                'total_requests': 0,
                'request_pattern': {},
                'preferred_endpoints': [],
                'avg_response_time': 0.0,
                'error_rate': 0.0,
                'timestamp': datetime.utcnow().isoformat()
            }

        # Calculate user-specific metrics
        total_requests = len(user_metrics)

        # Request patterns by hour
        hourly_pattern = defaultdict(int)
        for metric in user_metrics:
            hour = metric.timestamp.hour
            hourly_pattern[hour] += 1

        # Preferred endpoints
        endpoint_usage = defaultdict(int)
        for metric in user_metrics:
            endpoint_key = f"{metric.method} {metric.endpoint}"
            endpoint_usage[endpoint_key] += 1

        preferred_endpoints = sorted(endpoint_usage.items(), key=lambda x: x[1], reverse=True)[:5]

        # Calculate average response time for this user
        response_times = [metric.response_time for metric in user_metrics]
        avg_response_time = sum(response_times) / len(response_times) if response_times else 0.0

        # Calculate error rate for this user
        error_count = sum(1 for metric in user_metrics if metric.status_code >= 400)
        error_rate = (error_count / total_requests) * 100 if total_requests > 0 else 0.0

        return {
            'client_ip': client_ip,
            'total_requests': total_requests,
            'request_pattern': dict(hourly_pattern),
            'preferred_endpoints': preferred_endpoints,
            'avg_response_time': avg_response_time,
            'error_rate': error_rate,
            'timestamp': datetime.utcnow().isoformat()
        }