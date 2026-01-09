# P3-008: Prompt DSL Candidates - Evaluation Documentation

**Project**: ARIA Tool Execution MCP Team
**Version**: 1.0.0
**Date**: 2026-01-09
**Status**: Evaluation Phase

---

## Executive Summary

This document evaluates Domain-Specific Language (DSL) candidates for ARIA's prompt engineering, focusing on structured prompt composition, template management, and agent instruction standardization.

### Evaluation Scope
- **Prompt Templating**: DSLs for composing reusable prompt templates
- **Instruction Frameworks**: Structured formats for agent instructions
- **Prompt Optimization**: Tools for prompt engineering and testing
- **Multi-Agent Coordination**: Prompt patterns for agent collaboration

---

## 1. Prompt DSL Requirements

### 1.1 Core Requirements

```yaml
requirements:
  structure:
    - Hierarchical prompt composition
    - Variable interpolation
    - Conditional logic
    - Template inheritance
    - Modular sections

  safety:
    - Input validation
    - Output sanitization
    - Injection prevention
    - Content filtering

  observability:
    - Version tracking
    - A/B testing support
    - Performance metrics
    - Prompt debugging

  integration:
    - MCP tool compatibility
    - LLMOps pipeline integration
    - Multi-model support
    - Agent framework compatibility
```

### 1.2 ARIA-Specific Needs

```yaml
aria_needs:
  multi_agent:
    - Coordinator prompts
    - Domain specialist prompts
    - Cross-agent communication templates
    - Task delegation patterns

  domain_specific:
    - 20 domain templates (ARIA architecture)
    - Repository analysis prompts
    - Configuration management prompts
    - Workflow verification prompts

  context_management:
    - Large context handling
    - Context compression
    - Selective context injection
    - Memory-augmented prompts

  execution:
    - Tool usage instructions
    - Output format specifications
    - Error handling patterns
    - Retry logic templates
```

---

## 2. DSL Candidates

### 2.1 Prompt Markup Languages

#### 2.1.1 Guidance (Microsoft) - RECOMMENDED

**Overview:**
```yaml
name: Guidance
provider: Microsoft Research
type: "Python-embedded DSL"
license: MIT
status: "Active development"
```

**Strengths:**
- ✅ Python-native with intuitive syntax
- ✅ Strong type safety and validation
- ✅ Built-in token optimization
- ✅ Excellent model control
- ✅ Support for constrained generation
- ✅ Rich variable interpolation
- ✅ Conditional logic and loops

**Example:**
```python
from guidance import models, gen, select

# ARIA domain analysis prompt
guidance_model = models.LlamaCpp(model_path)

prompt = guidance_model + f"""
## ARIA Domain Analysis

**Domain:** {{domain}}
**Analysis Type:** {{analysis_type}}

### Instructions
Analyze the {{domain}} domain for:
{{#each focus_areas}}
- {{this}}
{{/each}}

### Repository Analysis
{{#if include_repos}}
Repositories to analyze:
{{gen 'repository_list' max_tokens=500}}
{{/if}}

### Output Format
{{#select 'output_format'}}
{{#option 'json'}}JSON structured output{{/option}}
{{#option 'markdown'}}Markdown report{{/option}}
{{/select}}

{{gen 'analysis' max_tokens=2000 temperature=0.7}}
"""
```

**Integration:**
```yaml
deployment:
  method: pixi
  package: guidance
  dependencies:
    - transformers
    - torch

use_cases:
  - Domain analysis templates
  - Configuration validation prompts
  - Structured output generation
  - Multi-step reasoning chains

integration_points:
  - LLMOps: TensorZero function calling
  - Memory: RAG context injection
  - MCP: Tool execution templates
```

**Priority:** P1 (High-value primary DSL)

---

#### 2.1.2 LMQL (Language Model Query Language)

**Overview:**
```yaml
name: LMQL
provider: ETH Zurich
type: "Standalone DSL"
license: Apache 2.0
status: "Active development"
```

**Strengths:**
- ✅ SQL-like syntax for prompts
- ✅ Strong constraints and validation
- ✅ Multi-model support
- ✅ Built-in debugging
- ✅ Type checking

**Example:**
```lmql
argmax
    "Analyze the [DOMAIN] domain:\n"
    "Repositories found: [REPOS]\n"
    "Installation method: [METHOD]\n"
    "Conflicts: [CONFLICTS]\n"
from
    "openai/gpt-4"
where
    DOMAIN in ["Agent Runtime", "Tool Execution", "Inference"]
    len(REPOS) < 500
    METHOD in ["nix", "pixi", "docker", "cargo"]
    CONFLICTS in ["none", "version", "dependency"]
```

**Integration:**
```yaml
deployment:
  method: pixi
  package: lmql
  complexity: "Higher learning curve"

use_cases:
  - Constrained generation
  - Formal verification
  - Complex query logic
```

**Weaknesses:**
- ⚠️ Steeper learning curve
- ⚠️ Less Python-native than Guidance
- ⚠️ Requires separate runtime

**Priority:** P2 (Alternative for specific use cases)

---

### 2.2 Prompt Template Frameworks

#### 2.2.1 Jinja2 (Current - Widely Used)

**Already Available** ✅

**Strengths:**
- ✅ Mature and battle-tested
- ✅ Excellent documentation
- ✅ Rich template features
- ✅ Large ecosystem
- ✅ Python-native

**Example:**
```jinja2
{# ARIA Domain Orchestration Template #}
## Domain Analysis: {{ domain }}

### Context
{% if context %}
{{ context }}
{% endif %}

### Analysis Scope
{% for focus_area in focus_areas %}
- {{ focus_area }}
{% endfor %}

### Repositories
{% if repositories %}
{% for repo in repositories %}
- **{{ repo.name }}**: {{ repo.description }}
  - URL: {{ repo.url }}
  - Stars: {{ repo.stars }}
  - Language: {{ repo.language }}
{% endfor %}
{% endif %}

### Task Instructions
{{ task_instructions }}

### Output Format
```{{ output_format }}
{{ output_schema | tojson(indent=2) }}
```
```

**Integration:**
```yaml
current_status: "Available in Python environment"
enhancement_needed: "Structured prompt library"

directory_structure:
  location: ".claude/prompts/"
  templates:
    - aria-orchestrator.md
    - coordinator prompts
    - domain specialist templates

enhancement_plan:
  - Create centralized template library
  - Add validation schemas
  - Implement version control
  - Add A/B testing support
```

**Priority:** P0 (Enhance existing usage)

---

#### 2.2.2 Handlebars

**Overview:**
```yaml
name: Handlebars
type: "Logic-less templates"
language: "JavaScript/Python ports"
license: MIT
```

**Strengths:**
- ✅ Simple and predictable
- ✅ Logic-less design
- ✅ Good for non-programmers
- ✅ Cross-language support

**Weaknesses:**
- ⚠️ Less powerful than Jinja2
- ⚠️ Limited logic capabilities

**Priority:** P3 (Only if cross-language needed)

---

### 2.3 Structured Prompt Frameworks

#### 2.3.1 Prompty (Microsoft) - RECOMMENDED

**Overview:**
```yaml
name: Prompty
provider: Microsoft
type: "Markdown-based prompt format"
license: MIT
status: "Active development"
integration: "VS Code extension available"
```

**Strengths:**
- ✅ Markdown-native format
- ✅ Built-in metadata and versioning
- ✅ LLMOps integration (PromptFlow)
- ✅ Model configuration in prompt
- ✅ Easy version control
- ✅ Human-readable

**Example:**
```markdown
---
name: ARIA Repository Analysis
description: Analyzes a GitHub repository for ARIA integration
authors:
  - ARIA Team
model:
  api: openai
  configuration:
    model: gpt-4
    temperature: 0.7
    max_tokens: 2000
sample:
  repository_url: "https://github.com/agixt/agixt"
  domain: "Agent Runtime"
  analysis_depth: "standard"
---

# Repository Analysis for ARIA

## Repository
{{repository_url}}

## Domain
{{domain}}

## Analysis Instructions

Analyze this repository for integration into the ARIA {{domain}} domain.

### Focus Areas
{{#each focus_areas}}
- {{this}}
{{/each}}

### Expected Output

```json
{
  "repository": {
    "url": "{{repository_url}}",
    "name": "string",
    "description": "string"
  },
  "installation_method": {
    "primary": "nix|pixi|docker|cargo",
    "target_file": "string"
  },
  "dependencies": {
    "system": [],
    "runtime": []
  },
  "conflicts": []
}
```
```

**Integration:**
```yaml
deployment:
  method: "File-based (.prompty files)"
  location: ".claude/prompts/"
  integration:
    - VS Code extension
    - PromptFlow for execution
    - Azure AI Studio

use_cases:
  - Version-controlled prompts
  - Collaborative prompt development
  - LLMOps pipeline integration
  - A/B testing framework

benefits:
  - Git-friendly format
  - Metadata tracking
  - Model configuration
  - Sample data for testing
```

**Priority:** P1 (High-value for prompt engineering)

---

#### 2.3.2 LangChain Prompt Templates

**Overview:**
```yaml
name: LangChain PromptTemplate
provider: LangChain
type: "Python prompt framework"
license: MIT
```

**Strengths:**
- ✅ Rich ecosystem integration
- ✅ Multiple template types
- ✅ Chat-specific templates
- ✅ Output parsers
- ✅ Few-shot learning support

**Example:**
```python
from langchain import PromptTemplate
from langchain.prompts import FewShotPromptTemplate

# ARIA domain analysis template
example_prompt = PromptTemplate(
    input_variables=["domain", "repos", "result"],
    template="""
    Domain: {domain}
    Repositories: {repos}
    Result: {result}
    """
)

aria_prompt = FewShotPromptTemplate(
    examples=[
        {
            "domain": "Agent Runtime",
            "repos": "agixt, autogen",
            "result": "docker-compose deployment"
        }
    ],
    example_prompt=example_prompt,
    prefix="Analyze the following domain for ARIA integration:",
    suffix="Domain: {domain}\nRepositories: {repos}\nResult:",
    input_variables=["domain", "repos"]
)
```

**Integration:**
```yaml
deployment:
  method: pixi
  package: langchain
  complexity: "Medium"

use_cases:
  - Agent chains
  - Few-shot learning
  - Output parsing
  - Memory-augmented prompts

integration_points:
  - Memory: Vector store retrievers
  - Tools: Agent toolkits
  - LLMOps: Chain execution
```

**Priority:** P1 (If using LangChain agents)

---

### 2.4 Prompt Optimization Frameworks

#### 2.4.1 DSPy (Stanford) - RECOMMENDED

**Overview:**
```yaml
name: DSPy
provider: Stanford NLP
type: "Prompt optimization framework"
license: MIT
status: "Active research & development"
```

**Strengths:**
- ✅ Automatic prompt optimization
- ✅ Program-based prompting
- ✅ Self-improving prompts
- ✅ Metric-driven optimization
- ✅ Module composition

**Concept:**
```python
import dspy

# Define ARIA analysis signature
class RepositoryAnalysis(dspy.Signature):
    """Analyze a repository for ARIA integration."""

    repository_url = dspy.InputField(desc="GitHub repository URL")
    domain = dspy.InputField(desc="ARIA domain")
    analysis_depth = dspy.InputField(desc="shallow|standard|deep")

    installation_method = dspy.OutputField(desc="Primary installation method")
    dependencies = dspy.OutputField(desc="List of dependencies")
    conflicts = dspy.OutputField(desc="Detected conflicts")

# DSPy automatically optimizes the prompt
class ARIAAnalyzer(dspy.Module):
    def __init__(self):
        super().__init__()
        self.analyze = dspy.ChainOfThought(RepositoryAnalysis)

    def forward(self, repository_url, domain, analysis_depth):
        return self.analyze(
            repository_url=repository_url,
            domain=domain,
            analysis_depth=analysis_depth
        )

# Optimize with examples
optimizer = dspy.BootstrapFewShot(metric=analysis_accuracy)
optimized_analyzer = optimizer.compile(
    ARIAAnalyzer(),
    trainset=training_examples
)
```

**Integration:**
```yaml
deployment:
  method: pixi
  package: dspy-ai

use_cases:
  - Self-improving prompts
  - Metric-driven optimization
  - Automatic few-shot selection
  - Program synthesis for prompts

benefits:
  - Reduces manual prompt engineering
  - Data-driven optimization
  - Compositional prompt modules
  - Metric tracking built-in

challenges:
  - Requires training data
  - Higher complexity
  - Newer framework
```

**Priority:** P2 (Research & optimization phase)

---

#### 2.4.2 PromptFoo (Testing)

**Overview:**
```yaml
name: PromptFoo
type: "Prompt testing and evaluation"
license: MIT
status: "Active development"
```

**Strengths:**
- ✅ Comprehensive testing framework
- ✅ Multi-model evaluation
- ✅ A/B testing support
- ✅ Regression testing
- ✅ CLI and CI/CD integration

**Example:**
```yaml
# promptfooconfig.yaml
description: "ARIA Repository Analysis Prompt Testing"

prompts:
  - aria-repository-analysis-v1.txt
  - aria-repository-analysis-v2.txt

providers:
  - openai:gpt-4
  - openai:gpt-4-turbo
  - anthropic:claude-sonnet-4.5

tests:
  - vars:
      repository_url: "https://github.com/agixt/agixt"
      domain: "Agent Runtime"
    assert:
      - type: contains-json
      - type: javascript
        value: "output.installation_method.primary in ['nix','pixi','docker','cargo']"
      - type: llm-rubric
        value: "Installation method is appropriate for the repository type"

  - vars:
      repository_url: "https://github.com/qdrant/qdrant"
      domain: "State & Storage"
    assert:
      - type: contains
        value: "docker"
      - type: cost
        threshold: 0.05
```

**Integration:**
```yaml
deployment:
  method: npm (via flake.nix wrapper)
  command: "npx promptfoo@latest"

use_cases:
  - Prompt regression testing
  - A/B testing
  - Multi-model evaluation
  - CI/CD integration

integration_points:
  - GitHub Actions workflows
  - Pre-commit hooks
  - Prompt version control
```

**Priority:** P1 (Essential for prompt quality)

---

### 2.5 Agent Communication DSLs

#### 2.5.1 FIPA ACL (Foundation for Intelligent Physical Agents)

**Overview:**
```yaml
name: FIPA ACL
type: "Agent Communication Language"
standard: "FIPA specification"
status: "Mature standard"
```

**Strengths:**
- ✅ Standardized agent communication
- ✅ Rich performative types
- ✅ Formal semantics
- ✅ Multi-agent coordination

**Performatives:**
```yaml
communicative_acts:
  inform: "Provide information"
  request: "Request action"
  query: "Ask for information"
  propose: "Propose action"
  accept_proposal: "Accept proposed action"
  reject_proposal: "Reject proposed action"
  agree: "Agree to perform action"
  refuse: "Refuse to perform action"
  subscribe: "Subscribe to information"
  cancel: "Cancel previous request"
```

**Example:**
```json
{
  "performative": "request",
  "sender": "coordinator_agent",
  "receiver": "domain_specialist_agent",
  "content": {
    "action": "analyze_domain",
    "parameters": {
      "domain": "Agent Runtime",
      "analysis_type": "comprehensive"
    }
  },
  "protocol": "aria_domain_analysis",
  "conversation_id": "conv_12345",
  "language": "JSON",
  "ontology": "aria_ontology_v1"
}
```

**Priority:** P2 (For formal multi-agent systems)

---

#### 2.5.2 AutoGen Conversation Patterns

**Overview:**
```yaml
name: AutoGen Patterns
provider: Microsoft Research
type: "Multi-agent conversation framework"
license: MIT
```

**Strengths:**
- ✅ Built-in conversation patterns
- ✅ Human-in-the-loop support
- ✅ Tool execution integration
- ✅ Code execution capabilities

**Patterns:**
```python
from autogen import AssistantAgent, UserProxyAgent, GroupChat, GroupChatManager

# ARIA multi-agent pattern
coordinator = AssistantAgent(
    name="ARIA_Coordinator",
    system_message="""You are the ARIA coordinator.
    Delegate domain analysis tasks to specialist agents.""",
    llm_config={"model": "gpt-4"}
)

domain_specialist = AssistantAgent(
    name="Domain_Specialist",
    system_message="""You analyze specific ARIA domains.
    Provide detailed repository analysis.""",
    llm_config={"model": "gpt-4"}
)

# Group chat for multi-agent coordination
groupchat = GroupChat(
    agents=[coordinator, domain_specialist],
    messages=[],
    max_round=10
)

manager = GroupChatManager(groupchat=groupchat)
```

**Priority:** P2 (If using AutoGen framework)

---

## 3. Recommended Stack

### 3.1 Primary Recommendation

```yaml
recommended_stack:
  templating:
    primary: "Jinja2"
    rationale: "Already available, mature, Python-native"
    priority: P0
    action: "Enhance with structured prompt library"

  structured_prompts:
    primary: "Prompty"
    rationale: "Git-friendly, metadata-rich, LLMOps integration"
    priority: P1
    action: "Adopt for new prompt development"

  dsl:
    primary: "Guidance"
    rationale: "Python-native, powerful, token-efficient"
    priority: P1
    action: "Deploy for complex prompt logic"

  testing:
    primary: "PromptFoo"
    rationale: "Comprehensive testing, CI/CD ready"
    priority: P1
    action: "Integrate into prompt development workflow"

  optimization:
    research: "DSPy"
    rationale: "Self-improving prompts, metric-driven"
    priority: P2
    action: "Evaluate for optimization phase"

  multi_agent:
    framework: "AutoGen patterns"
    rationale: "Microsoft-backed, agent-native"
    priority: P2
    action: "Evaluate for agent coordination"
```

---

### 3.2 Implementation Phases

#### Phase 1: Foundation (P0/P1)

```yaml
P0-1:
  title: "Structured Prompt Library with Jinja2"
  tasks:
    - Create .claude/prompts/templates/ directory
    - Migrate existing prompts to Jinja2 templates
    - Add validation schemas
    - Implement version control
  duration: "2-3 days"
  dependencies: []

P1-1:
  title: "Adopt Prompty Format"
  tasks:
    - Install VS Code Prompty extension
    - Convert key prompts to .prompty format
    - Add metadata and versioning
    - Integrate with Git workflow
  duration: "3-5 days"
  dependencies: [P0-1]

P1-2:
  title: "Deploy Guidance DSL"
  tasks:
    - Add guidance to pixi.toml
    - Create ARIA-specific Guidance modules
    - Implement token optimization
    - Document usage patterns
  duration: "5-7 days"
  dependencies: [P0-1]

P1-3:
  title: "Integrate PromptFoo Testing"
  tasks:
    - Add promptfoo to flake.nix
    - Create test configurations
    - Add to CI/CD pipeline
    - Set up regression tests
  duration: "3-5 days"
  dependencies: [P1-1]
```

#### Phase 2: Advanced Features (P2)

```yaml
P2-1:
  title: "DSPy Optimization Framework"
  tasks:
    - Install dspy-ai via pixi
    - Create optimization datasets
    - Implement metric functions
    - Run optimization experiments
  duration: "7-10 days"
  dependencies: [P1-2, P1-3]

P2-2:
  title: "Multi-Agent Communication Patterns"
  tasks:
    - Evaluate AutoGen vs custom patterns
    - Implement conversation templates
    - Add FIPA ACL support if needed
    - Test cross-agent coordination
  duration: "5-7 days"
  dependencies: [P1-2]
```

---

## 4. Prompt Engineering Best Practices

### 4.1 ARIA Prompt Structure

```yaml
standard_structure:
  metadata:
    - name
    - version
    - author
    - model_config
    - created_date
    - last_modified

  context:
    - system_context
    - domain_context
    - task_context

  instructions:
    - primary_objective
    - constraints
    - output_format
    - validation_criteria

  examples:
    - few_shot_examples
    - edge_cases

  memory_hooks:
    - retrieval_queries
    - context_injection_points

  tool_usage:
    - available_tools
    - tool_selection_criteria
    - error_handling
```

### 4.2 Token Optimization

```yaml
optimization_techniques:
  compression:
    - Remove unnecessary whitespace
    - Use abbreviations for repeated terms
    - Compress examples
    - Selective context injection

  caching:
    - Cache static instructions
    - Reuse system prompts
    - Semantic deduplication

  adaptive:
    - Adjust verbosity based on task
    - Dynamic example selection
    - Progressive detail levels
```

---

## 5. Evaluation Metrics

```yaml
metrics:
  quality:
    - Output relevance (NDCG)
    - Factual accuracy
    - Format compliance
    - Task completion rate

  performance:
    - Token efficiency (tokens/task)
    - Latency (ms/request)
    - Cost per task
    - Cache hit rate

  reliability:
    - Consistency across runs
    - Error rate
    - Fallback success rate
    - Robustness to variations

  maintainability:
    - Prompt version control
    - Test coverage
    - Documentation quality
    - Reusability score
```

---

## 6. Success Criteria

```yaml
success_criteria:
  phase1:
    - 100% of ARIA prompts templated
    - Jinja2 library with 20+ domain templates
    - Prompty format adopted for key workflows
    - PromptFoo integrated into CI/CD

  phase2:
    - Guidance DSL deployed for complex logic
    - Token usage reduced by 20%
    - Prompt test coverage > 80%
    - A/B testing framework operational

  phase3:
    - DSPy optimization showing 15%+ improvement
    - Multi-agent communication standardized
    - Self-improving prompt loops active
    - Comprehensive prompt observability
```

---

## 7. References

### Documentation
- Guidance: https://github.com/microsoft/guidance
- Prompty: https://github.com/microsoft/prompty
- PromptFoo: https://www.promptfoo.dev
- DSPy: https://github.com/stanfordnlp/dspy
- LangChain: https://python.langchain.com/docs/modules/model_io/prompts
- LMQL: https://lmql.ai

### Research
- "DSPy: Compiling Declarative Language Model Calls into Self-Improving Pipelines"
- "Language Models are Few-Shot Learners" (GPT-3 paper)
- "Chain-of-Thought Prompting Elicits Reasoning in Large Language Models"

### ARIA Context
- MANUS_ARIA_ORCHESTRATOR.md
- .claude/prompts/aria-orchestrator.md
- LLMOps documentation (docs/llmops/)

---

**Document Version**: 1.0.0
**Last Updated**: 2026-01-09
**Next Review**: 2026-02-09
**Owner**: ARIA Tool Execution MCP Team (L8 Team Lead)
