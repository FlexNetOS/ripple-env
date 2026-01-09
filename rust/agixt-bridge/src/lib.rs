//! AGiXT Bridge Library
//!
//! Provides types and utilities for ROS2-AGiXT integration.

pub mod client;
pub mod commands;
pub mod config;

pub use client::AgixtClient;
pub use commands::RobotCommand;
pub use config::BridgeConfig;

/// Re-export AGiXT SDK for convenience
pub use agixt_sdk;
