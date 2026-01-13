// AgentGateway MCP Routing - Main Application
// P3-001: Entry point for AgentGateway integration

use anyhow::Result;
use tracing::{info, error};
use clap::Parser;

/// AgentGateway MCP Router
#[derive(Parser, Debug)]
#[command(name = "agentgateway-router")]
#[command(about = "MCP routing for agentic OS", long_about = None)]
struct Args {
    /// Configuration file path
    #[arg(short, long, default_value = "config.yaml")]
    config: String,
    
    /// Enable debug logging
    #[arg(short, long)]
    debug: bool,
    
    /// Server bind address
    #[arg(short, long, default_value = "0.0.0.0:8080")]
    bind: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    
    // Initialize logging
    let log_level = if args.debug { "debug" } else { "info" };
    tracing_subscriber::fmt()
        .with_env_filter(format!("agentgateway={}", log_level))
        .init();
    
    info!("Starting AgentGateway MCP Router");
    info!("Configuration: {}", args.config);
    info!("Bind address: {}", args.bind);
    
    // Load configuration
    let config = load_config(&args.config)?;
    
    // Initialize router
    let router = agentgateway_routing::Router::new(config);
    
    // Start server
    match router.serve(&args.bind).await {
        Ok(_) => {
            info!("AgentGateway MCP Router started successfully");
            Ok(())
        }
        Err(e) => {
            error!("Failed to start server: {}", e);
            Err(e)
        }
    }
}

/// Load configuration from file
fn load_config(path: &str) -> Result<agentgateway_core::Config> {
    use std::fs;
    use serde_yaml;
    
    let contents = fs::read_to_string(path)?;
    let config: agentgateway_core::Config = serde_yaml::from_str(&contents)?;
    Ok(config)
}