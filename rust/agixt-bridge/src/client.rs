//! AGiXT Client wrapper for ROS2 integration

use agixt_sdk::AGiXTSDK;
use anyhow::{Context, Result};
use tracing::{info, warn};

use crate::config::BridgeConfig;

/// AGiXT client wrapper with LocalAI fallback support
pub struct AgixtClient {
    sdk: AGiXTSDK,
    config: BridgeConfig,
    connected: bool,
}

impl AgixtClient {
    /// Create a new AGiXT client
    pub fn new(config: BridgeConfig) -> Self {
        let sdk = AGiXTSDK::new(
            Some(config.agixt_url.clone()),
            Some(config.agixt_api_key.clone()),
            false,
        );

        Self {
            sdk,
            config,
            connected: false,
        }
    }

    /// Connect to AGiXT and verify availability
    pub async fn connect(&mut self) -> Result<()> {
        info!("Connecting to AGiXT at {}", self.config.agixt_url);

        match self.sdk.get_providers().await {
            Ok(providers) => {
                info!("AGiXT connected. Providers: {:?}", providers);
                self.connected = true;
                Ok(())
            }
            Err(e) => {
                warn!("AGiXT unavailable: {}. Trying LocalAI fallback...", e);
                self.verify_localai().await
            }
        }
    }

    /// Check LocalAI availability as fallback
    async fn verify_localai(&self) -> Result<()> {
        let client = reqwest::Client::new();
        let url = format!("{}/readyz", self.config.localai_url);

        client
            .get(&url)
            .send()
            .await
            .context("LocalAI not reachable")?;

        info!("LocalAI available at {}", self.config.localai_url);
        Ok(())
    }

    /// Get available AI providers
    pub async fn get_providers(&self) -> Result<Vec<String>> {
        let providers = self
            .sdk
            .get_providers()
            .await
            .context("Failed to get providers")?;

        Ok(providers
            .into_iter()
            .map(|p| format!("{:?}", p))
            .collect())
    }

    /// Create a new conversation
    pub async fn new_conversation(&self, agent: &str) -> Result<String> {
        let result = self
            .sdk
            .new_conversation(agent, None, None)
            .await
            .context("Failed to create conversation")?;

        Ok(format!("{:?}", result))
    }

    /// Check if connected to AGiXT
    pub fn is_connected(&self) -> bool {
        self.connected
    }

    /// Get the underlying SDK (for advanced usage)
    pub fn sdk(&self) -> &AGiXTSDK {
        &self.sdk
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_client_creation() {
        let config = BridgeConfig::default();
        let client = AgixtClient::new(config);
        assert!(!client.is_connected());
    }
}
