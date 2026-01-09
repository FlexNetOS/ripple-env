//! AGiXT-ROS2 Bridge
//!
//! Provides natural language command processing for ROS2 robots via AGiXT.
//!
//! # Usage
//!
//! ```bash
//! # Start LocalAI (on host)
//! localai start
//!
//! # Start AGiXT services (via Docker)
//! agixt up
//!
//! # Run the bridge
//! cargo run --bin agixt-bridge
//! ```

use agixt_sdk::AGiXTSDK;
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use tracing::{info, warn, Level};
use tracing_subscriber::FmtSubscriber;

/// Configuration for AGiXT bridge
#[derive(Debug, Deserialize)]
struct Config {
    /// AGiXT API base URL (default: http://localhost:7437)
    agixt_url: String,
    /// AGiXT API key
    agixt_api_key: String,
    /// LocalAI URL for direct inference fallback (default: http://localhost:8080)
    localai_url: String,
    /// Default agent name to use
    default_agent: String,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            agixt_url: std::env::var("AGIXT_URL")
                .unwrap_or_else(|_| "http://localhost:7437".to_string()),
            agixt_api_key: std::env::var("AGIXT_API_KEY")
                .unwrap_or_else(|_| "agixt-dev-key".to_string()),
            localai_url: std::env::var("LOCALAI_URL")
                .unwrap_or_else(|_| "http://localhost:8080".to_string()),
            default_agent: std::env::var("AGIXT_AGENT")
                .unwrap_or_else(|_| "ros2-agent".to_string()),
        }
    }
}

/// Robot command parsed from natural language
#[derive(Debug, Serialize, Deserialize)]
struct RobotCommand {
    /// Command type (move, rotate, stop, etc.)
    command_type: String,
    /// Target position or angle
    target: Option<f64>,
    /// Speed parameter
    speed: Option<f64>,
    /// Additional parameters
    params: std::collections::HashMap<String, serde_json::Value>,
}

/// AGiXT Bridge for ROS2
struct AgixtBridge {
    sdk: AGiXTSDK,
    config: Config,
}

impl AgixtBridge {
    /// Create a new AGiXT bridge
    fn new(config: Config) -> Self {
        let sdk = AGiXTSDK::new(
            Some(config.agixt_url.clone()),
            Some(config.agixt_api_key.clone()),
            false,
        );
        Self { sdk, config }
    }

    /// Initialize connection and verify AGiXT is available
    async fn init(&self) -> Result<()> {
        info!("Connecting to AGiXT at {}", self.config.agixt_url);

        // Try to get providers to verify connection
        match self.sdk.get_providers().await {
            Ok(providers) => {
                info!("AGiXT connected. Available providers: {:?}", providers);
                Ok(())
            }
            Err(e) => {
                warn!("AGiXT connection failed: {}. Using LocalAI fallback.", e);
                // Verify LocalAI is available as fallback
                self.verify_localai().await
            }
        }
    }

    /// Verify LocalAI is running
    async fn verify_localai(&self) -> Result<()> {
        let client = reqwest::Client::new();
        let url = format!("{}/readyz", self.config.localai_url);

        client
            .get(&url)
            .send()
            .await
            .context("LocalAI is not reachable")?;

        info!("LocalAI fallback available at {}", self.config.localai_url);
        Ok(())
    }

    /// Process natural language command and return robot command
    async fn process_command(&self, natural_language: &str) -> Result<RobotCommand> {
        info!("Processing command: {}", natural_language);

        // Create conversation with AGiXT
        let conversation = self
            .sdk
            .new_conversation(&self.config.default_agent, None, None)
            .await
            .context("Failed to create conversation")?;

        info!("Created conversation: {:?}", conversation);

        // For now, return a placeholder command
        // In a full implementation, this would parse AGiXT's response
        Ok(RobotCommand {
            command_type: "move".to_string(),
            target: Some(1.0),
            speed: Some(0.5),
            params: std::collections::HashMap::new(),
        })
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    let subscriber = FmtSubscriber::builder()
        .with_max_level(Level::INFO)
        .finish();
    tracing::subscriber::set_global_default(subscriber)?;

    // Load configuration
    dotenvy::dotenv().ok();
    let config = Config::default();

    info!("Starting AGiXT-ROS2 Bridge");
    info!("AGiXT URL: {}", config.agixt_url);
    info!("LocalAI URL: {}", config.localai_url);

    // Create and initialize bridge
    let bridge = AgixtBridge::new(config);
    bridge.init().await?;

    // Example: Process a command
    let command = bridge
        .process_command("Move forward 1 meter at half speed")
        .await?;
    info!("Parsed command: {:?}", command);

    info!("AGiXT bridge ready. Waiting for commands...");

    // In a full implementation, this would:
    // 1. Subscribe to a ROS2 topic for natural language commands
    // 2. Process each command via AGiXT
    // 3. Publish robot commands to appropriate ROS2 topics

    // Keep alive (in production, would be ROS2 spin)
    tokio::signal::ctrl_c().await?;
    info!("Shutting down AGiXT bridge");

    Ok(())
}
