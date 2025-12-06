# Performance Testing and Benchmarking Plan for RAG Chatbot Backend API

## Objective:
-   Verify that the backend API meets Non-Functional Requirement (NFR-002) which specifies performance goals (e.g., 1000 req/s, p95 latency < 200ms).
-   Identify performance bottlenecks and scalability limits.

## Scope:
-   `POST /embed` (for data ingestion performance)
-   `POST /query` (for retrieval latency and throughput)
-   `POST /ask` (for RAG generation latency and throughput)
-   `POST /selected_text` (for context-specific generation latency)
-   `POST /log` (for logging overhead)

## Tools:
-   **Load Testing**: Apache JMeter, k6, Locust, Hey.
-   **Monitoring**: Prometheus, Grafana (as outlined in `ops/monitoring/config.md`).

## Test Scenarios:

### 1. Baseline Performance:
-   **Concurrent Users**: 1
-   **Duration**: 5 minutes
-   **Ramp-up**: 1 minute
-   **Purpose**: Establish baseline latency and throughput for each endpoint.

### 2. Peak Load Testing:
-   **Concurrent Users**: Scaled up to target NFR (e.g., 100-1000 concurrent users).
-   **Duration**: 10-30 minutes
-   **Ramp-up**: Gradual increase to peak load.
-   **Purpose**: Verify system behavior under expected maximum load.

### 3. Stress Testing:
-   **Concurrent Users**: Beyond target NFR to identify breaking point.
-   **Duration**: Until system degradation or failure.
-   **Purpose**: Determine system's robustness and identify failure modes.

### 4. Soak Testing (Endurance Testing):
-   **Concurrent Users**: Moderate, sustained load.
-   **Duration**: Several hours (e.g., 4-8 hours).
-   **Purpose**: Detect memory leaks, resource exhaustion, or other long-term performance degradation.

## Metrics to Collect:
-   **Throughput**: Requests per second (RPS) for each endpoint.
-   **Latency**: Average, p50, p95, p99 response times.
-   **Error Rate**: Percentage of failed requests.
-   **Resource Utilization**: CPU, Memory, Network I/O (from system monitoring).

## Acceptance Criteria (referencing NFR-002 from plan.md):
-   `POST /query`: p95 latency < [NFR-002 value] ms under [X] RPS.
-   `POST /ask`: p95 latency < [NFR-002 value] ms under [X] RPS.
-   Overall error rate < [Y]%.
-   System remains stable without crashes or significant performance degradation during soak tests.

## Procedure:
1.  Set up the chosen load testing tool.
2.  Prepare test data (e.g., sample queries, contexts).
3.  Execute test scenarios.
4.  Monitor system resources and API performance during tests.
5.  Analyze results and generate a performance report.
