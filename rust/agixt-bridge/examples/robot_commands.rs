//! Robot command parsing example
//!
//! Shows how to parse natural language into robot commands.
//!
//! # Run
//!
//! ```bash
//! cargo run --example robot_commands
//! ```

use serde_json::json;
use std::collections::HashMap;

/// Robot command parsed from natural language
#[derive(Debug)]
struct RobotCommand {
    command_type: CommandType,
    target: Option<f64>,
    speed: Option<f64>,
    params: HashMap<String, serde_json::Value>,
}

#[derive(Debug, PartialEq)]
enum CommandType {
    Move,
    Rotate,
    Stop,
    GoTo,
    Unknown,
}

/// Simple command parser (in production, this would use AGiXT's NLU)
fn parse_command(input: &str) -> RobotCommand {
    let input_lower = input.to_lowercase();

    // Move commands
    if input_lower.contains("move") || input_lower.contains("go") {
        let distance = extract_number(&input_lower, &["meter", "meters", "m"]);
        let speed = extract_number(&input_lower, &["speed"])
            .or_else(|| {
                if input_lower.contains("fast") {
                    Some(1.0)
                } else if input_lower.contains("slow") {
                    Some(0.3)
                } else if input_lower.contains("half") {
                    Some(0.5)
                } else {
                    Some(0.5)
                }
            });

        return RobotCommand {
            command_type: CommandType::Move,
            target: distance.or(Some(1.0)),
            speed,
            params: HashMap::new(),
        };
    }

    // Rotate commands
    if input_lower.contains("rotate") || input_lower.contains("turn") {
        let angle = extract_number(&input_lower, &["degree", "degrees", "deg"]);
        let direction = if input_lower.contains("left") {
            -1.0
        } else {
            1.0
        };

        let mut params = HashMap::new();
        params.insert("direction".to_string(), json!(direction));

        return RobotCommand {
            command_type: CommandType::Rotate,
            target: angle.map(|a| a * direction).or(Some(90.0 * direction)),
            speed: Some(0.5),
            params,
        };
    }

    // Stop commands
    if input_lower.contains("stop") || input_lower.contains("halt") {
        return RobotCommand {
            command_type: CommandType::Stop,
            target: None,
            speed: None,
            params: HashMap::new(),
        };
    }

    // Unknown
    RobotCommand {
        command_type: CommandType::Unknown,
        target: None,
        speed: None,
        params: HashMap::new(),
    }
}

/// Extract a number from text (simple implementation)
fn extract_number(text: &str, units: &[&str]) -> Option<f64> {
    for unit in units {
        if let Some(pos) = text.find(unit) {
            let before = &text[..pos];
            let words: Vec<&str> = before.split_whitespace().collect();
            if let Some(last) = words.last() {
                if let Ok(num) = last.parse::<f64>() {
                    return Some(num);
                }
            }
        }
    }
    None
}

fn main() {
    println!("Robot Command Parser Example");
    println!("============================\n");

    let test_commands = vec![
        "Move forward 2 meters",
        "Go 1.5 meters at half speed",
        "Turn left 90 degrees",
        "Rotate right",
        "Stop immediately",
        "Move fast",
        "Go slow for 3 meters",
        "Unknown gibberish command",
    ];

    println!("Parsing test commands:\n");

    for cmd in test_commands {
        let parsed = parse_command(cmd);
        println!("Input:  \"{}\"", cmd);
        println!("Output: {:?}", parsed);
        println!();
    }

    println!("\nIn production, AGiXT would handle this parsing via LLM,");
    println!("providing much more sophisticated natural language understanding.");
    println!("\nExample AGiXT prompt for robot commands:");
    println!("---");
    println!("Parse the following command into a robot action:");
    println!("Command: {{user_input}}");
    println!("");
    println!("Return JSON with: command_type, target, speed, params");
    println!("Valid command_types: move, rotate, stop, goto, gripper_open, gripper_close");
    println!("---");
}
