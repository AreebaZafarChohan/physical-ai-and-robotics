from datetime import datetime
from typing import Dict, Any

# In a real-world scenario, these would be sent to a monitoring system
# like Prometheus, Datadog, or similar. For now, we'll keep them in memory
# or print to logs.

def record_query_response_time(endpoint: str, duration_ms: float, status_code: int):
    """
    Records the response time for a given query endpoint.
    """
    print(f"METRIC: query_response_time - endpoint={endpoint}, duration_ms={duration_ms:.2f}, status_code={status_code}")
    # In a real system:
    # send_metric_to_prometheus("query_response_time", {"endpoint": endpoint, "status_code": status_code}, duration_ms)

def record_rag_accuracy(query_id: str, accuracy_score: float, details: Dict[str, Any]):
    """
    Records RAG accuracy metrics. (More complex to implement accurately without user feedback)
    """
    print(f"METRIC: rag_accuracy - query_id={query_id}, score={accuracy_score:.2f}, details={details}")
    # In a real system:
    # send_metric_to_prometheus("rag_accuracy", {"query_id": query_id}, accuracy_score)

def record_api_usage(endpoint: str, method: str, success: bool, user_id: str = None):
    """
    Records API usage metrics.
    """
    print(f"METRIC: api_usage - endpoint={endpoint}, method={method}, success={success}, user_id={user_id}")
    # In a real system:
    # send_metric_to_prometheus("api_usage", {"endpoint": endpoint, "method": method, "success": success, "user_id": user_id}, 1)
