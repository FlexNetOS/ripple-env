//! Basic AGiXT chat example
//!
//! Demonstrates connecting to AGiXT and starting a conversation.
//!
//! # Prerequisites
//!
//! 1. Start LocalAI: `localai start`
//! 2. Start AGiXT: `agixt up`
//!
//! # Run
//!
//! ```bash
//! cargo run --example basic_chat
//! ```

use agixt_sdk::AGiXTSDK;
use anyhow::Result;

#[tokio::main]
async fn main() -> Result<()> {
    println!("AGiXT Basic Chat Example");
    println!("========================\n");

    // Configuration from environment
    let agixt_url =
        std::env::var("AGIXT_URL").unwrap_or_else(|_| "http://localhost:7437".to_string());
    let api_key =
        std::env::var("AGIXT_API_KEY").unwrap_or_else(|_| "agixt-dev-key".to_string());

    println!("Connecting to AGiXT at: {}", agixt_url);

    // Create SDK client
    let client = AGiXTSDK::new(Some(agixt_url), Some(api_key), false);

    // Get available providers
    println!("\nFetching available providers...");
    match client.get_providers().await {
        Ok(providers) => {
            println!("Available providers:");
            for provider in providers {
                println!("  - {:?}", provider);
            }
        }
        Err(e) => {
            println!("Warning: Could not fetch providers: {}", e);
            println!("Make sure AGiXT is running: agixt up");
        }
    }

    // Try to create a conversation
    let conversation_name = "basic-chat";
    println!(
        "\nCreating conversation with ros2-agent (name: {})...",
        conversation_name
    );
    match client
        .new_conversation("ros2-agent", conversation_name, None)
        .await
    {
        Ok(conversation) => {
            println!("Conversation created: {:?}", conversation);
        }
        Err(e) => {
            println!("Warning: Could not create conversation: {}", e);
            println!("The agent may not exist yet. Create it in the AGiXT UI.");
        }
    }

    println!("\nBasic chat example complete!");
    println!("\nNext steps:");
    println!("  1. Open AGiXT UI: http://localhost:3437");
    println!("  2. Create an agent named 'ros2-agent'");
    println!("  3. Configure it to use LocalAI provider");
    println!("  4. Run this example again");

    Ok(())
}
