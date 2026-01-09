//! Robot command types for ROS2 integration

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Robot command parsed from natural language via AGiXT
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotCommand {
    /// Command type identifier
    pub command_type: CommandType,
    /// Target value (position, angle, etc.)
    pub target: Option<f64>,
    /// Speed parameter (0.0 - 1.0)
    pub speed: Option<f64>,
    /// Duration in seconds
    pub duration: Option<f64>,
    /// Additional parameters
    pub params: HashMap<String, serde_json::Value>,
    /// Original natural language input
    pub original_input: String,
    /// Confidence score (0.0 - 1.0)
    pub confidence: f64,
}

/// Supported robot command types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum CommandType {
    /// Move forward/backward
    Move,
    /// Rotate in place
    Rotate,
    /// Stop all motion
    Stop,
    /// Go to specific coordinates
    GoTo,
    /// Follow a path
    FollowPath,
    /// Gripper open
    GripperOpen,
    /// Gripper close
    GripperClose,
    /// Arm move to position
    ArmMove,
    /// Camera look at
    CameraLook,
    /// Custom command
    Custom(String),
    /// Unknown/unparsed command
    Unknown,
}

impl RobotCommand {
    /// Create a new move command
    pub fn move_forward(distance: f64, speed: f64) -> Self {
        Self {
            command_type: CommandType::Move,
            target: Some(distance),
            speed: Some(speed),
            duration: None,
            params: HashMap::new(),
            original_input: String::new(),
            confidence: 1.0,
        }
    }

    /// Create a stop command
    pub fn stop() -> Self {
        Self {
            command_type: CommandType::Stop,
            target: None,
            speed: None,
            duration: None,
            params: HashMap::new(),
            original_input: String::new(),
            confidence: 1.0,
        }
    }

    /// Create a rotate command
    pub fn rotate(angle_degrees: f64, speed: f64) -> Self {
        Self {
            command_type: CommandType::Rotate,
            target: Some(angle_degrees),
            speed: Some(speed),
            duration: None,
            params: HashMap::new(),
            original_input: String::new(),
            confidence: 1.0,
        }
    }

    /// Set the original input string
    pub fn with_input(mut self, input: &str) -> Self {
        self.original_input = input.to_string();
        self
    }

    /// Set confidence score
    pub fn with_confidence(mut self, confidence: f64) -> Self {
        self.confidence = confidence.clamp(0.0, 1.0);
        self
    }

    /// Add a parameter
    pub fn with_param(mut self, key: &str, value: serde_json::Value) -> Self {
        self.params.insert(key.to_string(), value);
        self
    }

    /// Check if command is valid for execution
    pub fn is_valid(&self) -> bool {
        match self.command_type {
            CommandType::Move | CommandType::Rotate => self.target.is_some(),
            CommandType::Stop => true,
            CommandType::GoTo => self.params.contains_key("x") && self.params.contains_key("y"),
            CommandType::Unknown => false,
            _ => true,
        }
    }
}

impl Default for RobotCommand {
    fn default() -> Self {
        Self {
            command_type: CommandType::Unknown,
            target: None,
            speed: None,
            duration: None,
            params: HashMap::new(),
            original_input: String::new(),
            confidence: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_move_command() {
        let cmd = RobotCommand::move_forward(1.0, 0.5);
        assert_eq!(cmd.command_type, CommandType::Move);
        assert_eq!(cmd.target, Some(1.0));
        assert!(cmd.is_valid());
    }

    #[test]
    fn test_stop_command() {
        let cmd = RobotCommand::stop();
        assert_eq!(cmd.command_type, CommandType::Stop);
        assert!(cmd.is_valid());
    }

    #[test]
    fn test_unknown_command_invalid() {
        let cmd = RobotCommand::default();
        assert!(!cmd.is_valid());
    }
}
