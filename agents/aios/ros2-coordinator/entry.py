"""
ROS2 Coordinator Agent for AIOS
Coordinates robot actions through natural language commands.

Usage:
    pixi run -e aios python -m cerebrum run agents/aios/ros2-coordinator
"""

from cerebrum.client import Cerebrum

class ROS2CoordinatorAgent:
    """Agent that coordinates ROS2 robot actions."""

    def __init__(self):
        """Initialize the ROS2 coordinator agent."""
        self.client = Cerebrum()
        self.robot_state = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"yaw": 0.0},
            "velocity": {"linear": 0.0, "angular": 0.0},
            "battery": 100.0,
            "status": "idle"
        }

    def run(self, task: str) -> str:
        """
        Process a natural language command for robot control.

        Args:
            task: Natural language command (e.g., "Move forward 2 meters")

        Returns:
            Result of the command execution
        """
        # Parse the command using LLM
        parse_prompt = f"""
        You are a ROS2 robot command parser. Parse this command into an action.
        Command: {task}

        Current robot state:
        - Position: {self.robot_state['position']}
        - Status: {self.robot_state['status']}

        Return JSON with:
        - action: move|rotate|stop|status|navigate
        - params: action-specific parameters
        """

        response = self.client.llm_chat(
            messages=[{"role": "user", "content": parse_prompt}],
            model="localai/ggml-gpt4all-j",
            temperature=0.3
        )

        # Execute the parsed command
        try:
            import json
            action_data = json.loads(response)
            return self._execute_action(action_data)
        except json.JSONDecodeError:
            return f"Understood command: {task}. Parsed response: {response}"

    def _execute_action(self, action_data: dict) -> str:
        """Execute a parsed robot action."""
        action = action_data.get("action", "unknown")
        params = action_data.get("params", {})

        if action == "move":
            distance = params.get("distance", 1.0)
            self.robot_state["status"] = "moving"
            return f"Moving {distance} meters..."

        elif action == "rotate":
            angle = params.get("angle", 90.0)
            self.robot_state["status"] = "rotating"
            return f"Rotating {angle} degrees..."

        elif action == "stop":
            self.robot_state["status"] = "idle"
            return "Robot stopped."

        elif action == "status":
            return f"Robot status: {self.robot_state}"

        else:
            return f"Unknown action: {action}"


# Agent entry point for AIOS
agent = ROS2CoordinatorAgent()
