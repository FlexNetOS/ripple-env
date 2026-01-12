# TruLens Configuration

LLM evaluation and testing configuration using TruLens.

## Overview

TruLens provides comprehensive evaluation capabilities for:
- LLM responses
- RAG (Retrieval-Augmented Generation) systems
- AI agents
- Multi-model pipelines

## Setup

### 1. Install TruLens

Already configured in `pixi.toml`:

```bash
pixi install
```

### 2. Configure Environment

```bash
# Copy example environment file
cp config/trulens/.env.example .env

# Edit with your API keys
export OPENAI_API_KEY=sk-...
export ANTHROPIC_API_KEY=sk-ant-...
```

### 3. Start TruLens Dashboard

```bash
# Start the dashboard
trulens-eval run-dashboard --config config/trulens/trulens_config.yaml

# Or use Python API
python -c "from trulens_eval import Tru; Tru().run_dashboard()"
```

Access at: http://localhost:8501

## Usage

### Basic Evaluation

```python
from trulens_eval import Tru, TruBasicApp, Feedback, Select
from trulens_eval.feedback.provider import OpenAI

# Initialize TruLens
tru = Tru()

# Define feedback functions
provider = OpenAI()
f_relevance = Feedback(provider.relevance).on_input_output()
f_groundedness = Feedback(provider.groundedness_measure_with_cot_reasons).on_output()

# Wrap your LLM application
recorded_app = TruBasicApp(
    your_llm_function,
    app_id="my-llm-app",
    feedbacks=[f_relevance, f_groundedness]
)

# Run and record
with recorded_app as recording:
    result = your_llm_function("What is Kubernetes?")

# View results in dashboard
tru.run_dashboard()
```

### RAG Evaluation

```python
from trulens_eval import Tru, TruChain, Feedback
from trulens_eval.feedback.provider import OpenAI

provider = OpenAI()

# RAG-specific feedback
f_qa_relevance = Feedback(provider.relevance).on_input_output()
f_context_relevance = Feedback(provider.qs_relevance).on_input().on(
    Select.RecordCalls.retrieve.rets
)
f_groundedness = Feedback(provider.groundedness_measure_with_cot_reasons).on(
    Select.RecordCalls.retrieve.rets.collect()
).on_output()

# Wrap your RAG chain
tru_rag = TruChain(
    your_rag_chain,
    app_id="my-rag-app",
    feedbacks=[f_qa_relevance, f_context_relevance, f_groundedness]
)

# Run evaluation
with tru_rag as recording:
    result = your_rag_chain.invoke({"question": "What is Kubernetes?"})
```

### Agent Evaluation

```python
from trulens_eval import Tru, TruCustomApp, Feedback
from trulens_eval.feedback.provider import OpenAI

provider = OpenAI()

# Agent-specific feedback
f_tool_selection = Feedback(provider.relevance).on(
    Select.RecordCalls.agent.rets.tool
).on_input()

f_task_completion = Feedback(provider.task_completion).on_input_output()

# Wrap your agent
tru_agent = TruCustomApp(
    your_agent,
    app_id="my-agent",
    feedbacks=[f_tool_selection, f_task_completion]
)
```

## Feedback Functions

### Available Metrics

1. **Relevance**: Answer relevance to question (0-1)
2. **Groundedness**: Answer grounded in context (0-1)
3. **Context Relevance**: Context relevance to question (0-1)
4. **Toxicity**: Toxic content detection (0-1, lower is better)
5. **Coherence**: Response coherence (0-1)
6. **Conciseness**: Response brevity (0-1)

### Custom Feedback

```python
from trulens_eval import Feedback

def custom_metric(input, output):
    """Custom evaluation metric"""
    # Your logic here
    return score  # 0-1

f_custom = Feedback(custom_metric).on_input_output()
```

## Experiments

### Running Experiments

```python
# Tag experiments with metadata
with tru.session(
    experiment_id="rag-v2",
    tags=["rag", "production"],
    metadata={"model": "gpt-4", "version": "2.0"}
):
    # Run your evaluations
    pass
```

### Comparing Experiments

```python
# Get all experiments
experiments = tru.get_records()

# Compare specific experiments
exp1 = tru.get_records(app_ids=["rag-v1"])
exp2 = tru.get_records(app_ids=["rag-v2"])

# Generate comparison report
from trulens_eval import utils
utils.compare_experiments([exp1, exp2])
```

## Integration

### Pixi Configuration

Already added to `pixi.toml`:

```toml
[dependencies]
trulens-eval = ">=1.0.0"
```

### CI/CD Integration

```yaml
# GitHub Actions example
- name: Run LLM Evaluations
  run: |
    pixi run python scripts/run-trulens-eval.py

- name: Check Evaluation Thresholds
  run: |
    python scripts/check-eval-thresholds.py \
      --min-relevance 0.7 \
      --min-groundedness 0.7
```

## Data Storage

Evaluation data is stored in:

```
data/trulens/
├── trulens.db          # SQLite database
├── experiments/        # Experiment metadata
├── cache/             # Cached evaluations
└── trulens.log        # Application logs
```

For production, configure PostgreSQL:

```yaml
database:
  type: postgresql
  url: postgresql://user:pass@postgres:5432/trulens
```

## Monitoring

### Prometheus Metrics

TruLens exports metrics to Prometheus:

```yaml
# prometheus.yml
scrape_configs:
  - job_name: 'trulens'
    static_configs:
      - targets: ['localhost:9090']
```

### Grafana Dashboard

Import the TruLens dashboard template from `config/trulens/grafana-dashboard.json`

## Best Practices

1. **Threshold-based Gating**: Set minimum scores for deployment
2. **Continuous Evaluation**: Run evaluations on every commit
3. **Experiment Tracking**: Tag and version all experiments
4. **Cost Management**: Use local models for bulk evaluations
5. **Feedback Diversity**: Use multiple metrics for robust evaluation

## Troubleshooting

### Dashboard not starting

```bash
# Check if port is in use
lsof -i :8501

# Use different port
trulens-eval run-dashboard --port 8502
```

### API Key Issues

```bash
# Verify API keys
echo $OPENAI_API_KEY
echo $ANTHROPIC_API_KEY

# Test API connection
python -c "from trulens_eval.feedback.provider import OpenAI; OpenAI().endpoint.client.models.list()"
```

### Database Errors

```bash
# Reset database
rm data/trulens/trulens.db
python -c "from trulens_eval import Tru; Tru().reset_database()"
```

## Resources

- [TruLens Documentation](https://www.trulens.org/)
- [TruLens GitHub](https://github.com/truera/trulens)
- [Feedback Functions Reference](https://www.trulens.org/trulens_eval/api/feedback/)
- [RAG Evaluation Guide](https://www.trulens.org/trulens_eval/rag_triad/)

## Related Files

- [trulens_config.yaml](./trulens_config.yaml) - Main configuration
- [pixi.toml](../../pixi.toml) - Package dependencies
- [scripts/run-trulens-eval.py](../../scripts/run-trulens-eval.py) - Evaluation script
