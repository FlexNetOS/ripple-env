# Distributed Tracing with Grafana Tempo

## Overview

This document describes the distributed tracing setup using **Grafana Tempo** integrated with the observability stack. Tempo provides high-volume, cost-effective distributed tracing that integrates seamlessly with Prometheus, Loki, and Grafana.

## Architecture

### Components

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ LocalAI  │  │  AGiXT   │  │Holochain │  │  ROS2    │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘   │
│       │             │              │              │         │
│       └─────────────┴──────────────┴──────────────┘         │
│                          │                                   │
│                  OTLP (gRPC/HTTP)                           │
└──────────────────────────┼──────────────────────────────────┘
                           │
                           ▼
        ┌──────────────────────────────────────┐
        │    OpenTelemetry Collector           │
        │  - Receives OTLP traces              │
        │  - Processes & enriches              │
        │  - Routes to Tempo                   │
        └──────────────┬───────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────────────┐
        │         Grafana Tempo                │
        │  - Stores traces (local/S3)          │
        │  - Generates metrics from traces     │
        │  - Service graphs & dependencies     │
        └──────────────┬───────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────────────┐
        │            Grafana                   │
        │  - Trace visualization               │
        │  - Trace-to-logs correlation         │
        │  - Trace-to-metrics correlation      │
        │  - Service dependency maps           │
        └──────────────────────────────────────┘
```

### Data Flow

1. **Application Instrumentation**: Applications use OpenTelemetry SDKs to generate traces
2. **Collection**: OTel Collector receives traces via OTLP (gRPC or HTTP)
3. **Processing**: Collector adds metadata, batches, and forwards to Tempo
4. **Storage**: Tempo stores traces and generates derived metrics
5. **Querying**: Grafana queries Tempo for trace visualization and analysis

## Configuration

### Tempo Configuration

Location: `/home/user/ripple-env/config/tempo/tempo.yaml`

Key features:
- **OTLP receivers**: Accept traces from OpenTelemetry instrumented applications
- **Jaeger receivers**: Support legacy Jaeger clients
- **Local storage**: Stores traces on disk (configurable to S3/GCS)
- **Metrics generation**: Automatically creates RED metrics (Rate, Errors, Duration)
- **Service graphs**: Builds dependency graphs from trace data

### OTel Collector Configuration

Location: `/home/user/ripple-env/manifests/observability/otel-collector.yaml`

Traces pipeline:
```yaml
traces:
  receivers: [otlp]
  processors: [memory_limiter, batch, resource]
  exporters: [otlp/tempo, debug]
```

### Grafana Datasource

Location: `/home/user/ripple-env/manifests/observability/grafana/provisioning/datasources/datasources.yml`

Features enabled:
- **Trace-to-logs**: Jump from traces to related logs in Loki
- **Trace-to-metrics**: View metrics for traced operations in Prometheus
- **Service maps**: Visualize service dependencies
- **Node graphs**: Interactive dependency exploration

## Deployment

### Start the Observability Stack

```bash
cd /home/user/ripple-env

# Create network if it doesn't exist
docker network create agentic-network 2>/dev/null || true

# Start all observability services
docker compose -f docker/docker-compose.observability.yml up -d

# Verify Tempo is running
docker compose -f docker/docker-compose.observability.yml ps tempo
docker compose -f docker/docker-compose.observability.yml logs tempo
```

### Verify Services

```bash
# Check Tempo health
curl http://localhost:3200/ready

# Check OTel Collector is forwarding to Tempo
curl http://localhost:13133/

# View Tempo configuration
curl http://localhost:3200/config
```

### Access Points

| Service | URL | Purpose |
|---------|-----|---------|
| Tempo API | http://localhost:3200 | Query traces, configuration |
| Grafana | http://localhost:3000 | Visualize traces (admin/admin) |
| OTel Collector | http://localhost:4317 (gRPC) | Send OTLP traces |
| OTel Collector | http://localhost:4318 (HTTP) | Send OTLP traces |

## Instrumentation

### Python Applications

```python
from opentelemetry import trace
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.sdk.resources import Resource

# Configure resource attributes
resource = Resource(attributes={
    "service.name": "my-service",
    "service.namespace": "flexstack",
    "deployment.environment": "development"
})

# Set up tracer provider
provider = TracerProvider(resource=resource)
trace.set_tracer_provider(provider)

# Configure OTLP exporter
otlp_exporter = OTLPSpanExporter(
    endpoint="http://localhost:4317",
    insecure=True
)

# Add batch processor
provider.add_span_processor(BatchSpanProcessor(otlp_exporter))

# Get tracer
tracer = trace.get_tracer(__name__)

# Create spans
with tracer.start_as_current_span("operation-name") as span:
    span.set_attribute("custom.attribute", "value")
    # Your code here
```

### ROS2 Nodes

```python
# In your ROS2 node
import rclpy
from rclpy.node import Node
from opentelemetry import trace

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.tracer = trace.get_tracer(__name__)

    def timer_callback(self):
        with self.tracer.start_as_current_span("process_data") as span:
            span.set_attribute("node.name", self.get_name())
            # Process data
```

### JavaScript/Node.js

```javascript
const { NodeTracerProvider } = require('@opentelemetry/sdk-trace-node');
const { OTLPTraceExporter } = require('@opentelemetry/exporter-trace-otlp-grpc');
const { BatchSpanProcessor } = require('@opentelemetry/sdk-trace-base');
const { Resource } = require('@opentelemetry/resources');
const { SemanticResourceAttributes } = require('@opentelemetry/semantic-conventions');

const resource = Resource.default().merge(
  new Resource({
    [SemanticResourceAttributes.SERVICE_NAME]: "my-service",
    [SemanticResourceAttributes.SERVICE_NAMESPACE]: "flexstack",
  })
);

const provider = new NodeTracerProvider({ resource });
const exporter = new OTLPTraceExporter({
  url: 'http://localhost:4317',
});

provider.addSpanProcessor(new BatchSpanProcessor(exporter));
provider.register();
```

## Using Grafana for Trace Analysis

### Accessing Traces

1. Open Grafana: http://localhost:3000
2. Navigate to **Explore** (compass icon)
3. Select **Tempo** datasource
4. Choose query type:
   - **Search**: Find traces by tags, duration, etc.
   - **TraceQL**: Use TraceQL query language
   - **Service Graph**: View service dependencies
   - **Upload JSON**: Import trace files

### Search Queries

```traceql
# Find all traces for a specific service
{ service.name = "localai" }

# Find slow traces (> 1 second)
{ duration > 1s }

# Find errors
{ status = error }

# Complex query: slow errors in production
{
  service.namespace = "flexstack" &&
  deployment.environment = "production" &&
  status = error &&
  duration > 500ms
}

# Find traces with specific HTTP status
{ http.status_code = 500 }

# Find traces by span name
{ name = "database_query" }
```

### Trace-to-Logs Correlation

When viewing a trace:
1. Click on any span
2. Click **Logs for this span** button
3. See related logs from Loki filtered by:
   - Time range (span duration ± 1 hour)
   - Trace ID
   - Service name

### Trace-to-Metrics Correlation

1. Select a span in trace view
2. Click **Metrics** tab
3. View RED metrics for the span's operation:
   - **Rate**: Requests per second
   - **Errors**: Error percentage
   - **Duration**: Latency percentiles (p50, p90, p99)

### Service Graph

1. In Explore, select Tempo datasource
2. Click **Service Graph** tab
3. View:
   - Service nodes
   - Request rates between services
   - Error rates
   - Latency

## Metrics Generation

Tempo automatically generates metrics from traces:

### Span Metrics

Available in Prometheus:
```promql
# Request rate by service
rate(traces_spanmetrics_calls_total[5m])

# Error rate by service
rate(traces_spanmetrics_calls_total{status_code="STATUS_CODE_ERROR"}[5m])
/ rate(traces_spanmetrics_calls_total[5m])

# Latency percentiles
histogram_quantile(0.95,
  rate(traces_spanmetrics_latency_bucket[5m])
)
```

### Service Graph Metrics

```promql
# Request rate between services
traces_service_graph_request_total

# Failed requests between services
traces_service_graph_request_failed_total

# Client-side latency
traces_service_graph_request_client_seconds

# Server-side latency
traces_service_graph_request_server_seconds
```

## Best Practices

### Trace Sampling

For high-volume services, implement sampling:

```python
from opentelemetry.sdk.trace.sampling import TraceIdRatioBased

# Sample 10% of traces
sampler = TraceIdRatioBased(0.1)
provider = TracerProvider(sampler=sampler, resource=resource)
```

### Span Attributes

Use semantic conventions:

```python
from opentelemetry.semconv.trace import SpanAttributes

with tracer.start_as_current_span("http_request") as span:
    span.set_attribute(SpanAttributes.HTTP_METHOD, "GET")
    span.set_attribute(SpanAttributes.HTTP_URL, "https://api.example.com")
    span.set_attribute(SpanAttributes.HTTP_STATUS_CODE, 200)
```

### Error Handling

```python
with tracer.start_as_current_span("operation") as span:
    try:
        # Your code
        result = do_something()
    except Exception as e:
        span.set_status(Status(StatusCode.ERROR, str(e)))
        span.record_exception(e)
        raise
```

### Context Propagation

For distributed traces across services:

```python
from opentelemetry import propagate
from opentelemetry.propagators.b3 import B3MultiFormat

# Set B3 propagator for compatibility with Zipkin/Jaeger
propagate.set_global_textmap(B3MultiFormat())

# In HTTP requests
headers = {}
propagate.inject(headers)
requests.get(url, headers=headers)

# In HTTP handlers
ctx = propagate.extract(request.headers)
with tracer.start_as_current_span("handler", context=ctx):
    # Your handler code
```

## Troubleshooting

### No Traces Appearing

1. **Check OTel Collector logs**:
   ```bash
   docker logs otel-collector
   ```

2. **Verify traces are being sent**:
   ```bash
   # Check debug exporter output
   docker logs otel-collector | grep -A 10 "Traces"
   ```

3. **Check Tempo logs**:
   ```bash
   docker logs tempo
   ```

4. **Verify Tempo is receiving traces**:
   ```bash
   # Check ingestion metrics
   curl http://localhost:3200/metrics | grep tempo_ingester
   ```

### High Memory Usage

Adjust Tempo configuration:

```yaml
ingester:
  max_block_bytes: 500_000  # Reduce from 1_000_000
  max_block_duration: 2m    # Reduce from 5m
```

### Slow Queries

1. **Enable caching** in Grafana datasource
2. **Use time range filters** in queries
3. **Index traces** for frequently queried tags

### Port Conflicts

If Tempo's OTLP ports conflict with OTel Collector:

```yaml
# docker-compose.observability.yml - Tempo service
ports:
  - "3200:3200"   # Keep Tempo API
  # Remove or remap conflicting ports
  - "14317:4317"  # Map to different host port
  - "14318:4318"
```

## Storage Management

### Local Storage

Default path: `/tmp/tempo/blocks`

Retention: 7 days (168 hours)

```bash
# Check storage usage
docker exec tempo du -sh /tmp/tempo/blocks

# View trace blocks
docker exec tempo ls -lh /tmp/tempo/blocks
```

### S3 Backend (Production)

For production, configure S3 storage:

```yaml
storage:
  trace:
    backend: s3
    s3:
      bucket: tempo-traces
      endpoint: s3.amazonaws.com
      region: us-east-1
    wal:
      path: /tmp/tempo/wal
```

## Performance Tuning

### Ingestion Rate

```yaml
# Increase parallelism for high-volume traces
ingester:
  max_block_bytes: 5_000_000
  max_block_duration: 10m

distributor:
  ring:
    kvstore:
      store: inmemory
```

### Query Performance

```yaml
query_frontend:
  search:
    concurrent_jobs: 2000
    max_duration: 0s  # No limit
  trace_by_id:
    query_shards: 50
```

## Integration Examples

### AGiXT Integration

AGiXT can send traces for agent operations:

```python
# In AGiXT agent
with tracer.start_as_current_span("agent_execute") as span:
    span.set_attribute("agent.name", agent_name)
    span.set_attribute("task.type", task_type)

    result = execute_task(task)

    span.set_attribute("result.status", "success")
```

### LocalAI Integration

LocalAI inference traced:

```python
with tracer.start_as_current_span("inference") as span:
    span.set_attribute("model.name", model_name)
    span.set_attribute("input.tokens", len(tokens))

    response = model.generate(prompt)

    span.set_attribute("output.tokens", len(response))
    span.set_attribute("inference.duration_ms", duration)
```

### Holochain Integration

Trace DHT operations:

```rust
use opentelemetry::trace::Tracer;

let tracer = global::tracer("holochain");
let span = tracer.start("dht_get");
span.set_attribute("entry.hash", hash.to_string());

let result = dht_get(hash).await;

span.set_attribute("result.found", result.is_some());
span.end();
```

## Security Considerations

### Network Security

- Tempo uses **insecure gRPC** by default in development
- For production, enable **TLS/mTLS**:

```yaml
server:
  grpc_tls_config:
    cert_file: /etc/tempo/tls/server.crt
    key_file: /etc/tempo/tls/server.key
    client_ca_file: /etc/tempo/tls/ca.crt
```

### Access Control

- Use **Kong** or **Keycloak** to control access to Tempo API
- Implement **tenant isolation** for multi-tenant deployments

### Data Privacy

- **Scrub sensitive data** before sending traces:

```python
# Remove PII from attributes
with tracer.start_as_current_span("user_operation") as span:
    span.set_attribute("user.id", hash_user_id(user_id))
    # Don't include: email, password, tokens, etc.
```

## Resources

- [Grafana Tempo Documentation](https://grafana.com/docs/tempo/latest/)
- [OpenTelemetry Python](https://opentelemetry.io/docs/instrumentation/python/)
- [OpenTelemetry JavaScript](https://opentelemetry.io/docs/instrumentation/js/)
- [TraceQL Documentation](https://grafana.com/docs/tempo/latest/traceql/)
- [Service Graph Documentation](https://grafana.com/docs/tempo/latest/metrics-generator/service-graphs/)
- [Span Metrics Documentation](https://grafana.com/docs/tempo/latest/metrics-generator/span_metrics/)

## Next Steps

1. **Instrument critical services**: Start with LocalAI, AGiXT, and ROS2 nodes
2. **Create dashboards**: Build Grafana dashboards for trace analytics
3. **Set up alerts**: Alert on high error rates or latency from trace metrics
4. **Configure sampling**: Implement smart sampling for high-volume services
5. **Migrate to S3**: Move to S3/GCS storage for production
6. **Enable exemplars**: Link Prometheus metrics to traces with exemplars
