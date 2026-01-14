"""
DevOps Assistant Agent for AIOS
Assists with CI/CD, infrastructure, and automation tasks.

Usage:
    pixi run -e aios python -m cerebrum run agents/aios/devops-assistant
"""

from cerebrum.client import Cerebrum

class DevOpsAssistantAgent:
    """Agent that assists with DevOps tasks."""

    def __init__(self):
        """Initialize the DevOps assistant agent."""
        self.client = Cerebrum()
        self.context = {
            "platform": "nix-flakes",
            "ci": "github-actions",
            "container": "docker",
            "orchestration": "kubernetes"
        }

    def run(self, task: str) -> str:
        """
        Process a DevOps task request.

        Args:
            task: Description of the DevOps task

        Returns:
            Result or guidance for the task
        """
        analysis_prompt = f"""
        You are a DevOps expert. Analyze this task and provide guidance.

        Task: {task}

        Environment:
        - Platform: {self.context['platform']}
        - CI/CD: {self.context['ci']}
        - Container: {self.context['container']}
        - Orchestration: {self.context['orchestration']}

        Provide:
        1. Task classification (deployment, ci-cd, infrastructure, monitoring, security)
        2. Recommended approach
        3. Key commands or configurations needed
        4. Potential risks and mitigations

        Format as structured guidance.
        """

        response = self.client.llm_chat(
            messages=[{"role": "user", "content": analysis_prompt}],
            model="localai/ggml-gpt4all-j",
            temperature=0.5
        )

        # Store in memory for context
        self.client.memory_write(
            key=f"devops_task_{hash(task)}",
            value={
                "task": task,
                "response": response,
                "context": self.context
            }
        )

        return response

    def generate_script(self, task: str, script_type: str = "bash") -> str:
        """
        Generate a script for a DevOps task.

        Args:
            task: Description of what the script should do
            script_type: Type of script (bash, yaml, nix)

        Returns:
            Generated script content
        """
        script_prompt = f"""
        Generate a {script_type} script for this task:
        {task}

        Requirements:
        - Include error handling
        - Add comments
        - Follow security best practices
        - Use environment variables for secrets
        """

        response = self.client.llm_chat(
            messages=[{"role": "user", "content": script_prompt}],
            model="localai/ggml-gpt4all-j",
            temperature=0.3
        )

        return response


# Agent entry point for AIOS
agent = DevOpsAssistantAgent()
