from prometheus_client import Gauge, generate_latest, Counter, Histogram
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response
import time

# Define Prometheus metrics
REQUEST_COUNT = Counter(
    'http_requests_total', 'Total HTTP requests', ['method', 'endpoint', 'status_code']
)
REQUEST_LATENCY = Histogram(
    'http_request_duration_seconds', 'HTTP request latency', ['method', 'endpoint']
)

class PrometheusMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next):
        method = request.method
        endpoint = request.url.path

        start_time = time.time()
        response = await call_next(request)
        process_time = time.time() - start_time

        status_code = response.status_code

        # Increment request counter
        REQUEST_COUNT.labels(method=method, endpoint=endpoint, status_code=status_code).inc()
        # Observe request latency
        REQUEST_LATENCY.labels(method=method, endpoint=endpoint).observe(process_time)

        return response

def metrics_endpoint(request: Request):
    """
    Exposes Prometheus metrics.
    """
    return Response(generate_latest(), media_type="text/plain")

if __name__ == "__main__":
    # Example usage for testing purposes
    print("Prometheus metrics configured. Use PrometheusMiddleware in your FastAPI app.")
    print("Example endpoint for metrics: /metrics")
