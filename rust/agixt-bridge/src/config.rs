//! Configuration for AGiXT Bridge

use serde::Deserialize;

/// Bridge configuration loaded from environment or config file
#[derive(Debug, Clone, Deserialize)]
pub struct BridgeConfig {
    /// AGiXT API base URL
    #[serde(default = "default_agixt_url")]
    pub agixt_url: String,

    /// AGiXT API key
    #[serde(default = "default_agixt_api_key")]
    pub agixt_api_key: String,

    /// LocalAI URL for direct inference fallback
    #[serde(default = "default_localai_url")]
    pub localai_url: String,

    /// Default agent name
    #[serde(default = "default_agent")]
    pub default_agent: String,

    /// Connection timeout in seconds
    #[serde(default = "default_timeout")]
    pub timeout_secs: u64,

    /// Enable debug logging
    #[serde(default)]
    pub debug: bool,

    /// ROS2 node name (when ros2 feature enabled)
    #[serde(default = "default_ros2_node")]
    pub ros2_node_name: String,

    /// ROS2 namespace
    #[serde(default)]
    pub ros2_namespace: String,
}

fn default_agixt_url() -> String {
    std::env::var("AGIXT_URL").unwrap_or_else(|_| "http://localhost:7437".to_string())
}

fn default_agixt_api_key() -> String {
    std::env::var("AGIXT_API_KEY").unwrap_or_else(|_| "agixt-dev-key".to_string())
}

fn default_localai_url() -> String {
    std::env::var("LOCALAI_URL").unwrap_or_else(|_| "http://localhost:8080".to_string())
}

fn default_agent() -> String {
    std::env::var("AGIXT_AGENT").unwrap_or_else(|_| "ros2-agent".to_string())
}

fn default_timeout() -> u64 {
    30
}

fn default_ros2_node() -> String {
    "agixt_bridge".to_string()
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            agixt_url: default_agixt_url(),
            agixt_api_key: default_agixt_api_key(),
            localai_url: default_localai_url(),
            default_agent: default_agent(),
            timeout_secs: default_timeout(),
            debug: false,
            ros2_node_name: default_ros2_node(),
            ros2_namespace: String::new(),
        }
    }
}

impl BridgeConfig {
    /// Load configuration from environment variables
    pub fn from_env() -> Self {
        Self::default()
    }

    /// Load configuration from a TOML file
    pub fn from_file(path: &str) -> Result<Self, config::ConfigError> {
        let settings = config::Config::builder()
            .add_source(config::File::with_name(path))
            .add_source(config::Environment::with_prefix("AGIXT_BRIDGE"))
            .build()?;

        settings.try_deserialize()
    }

    /// Create config with custom AGiXT URL
    pub fn with_agixt_url(mut self, url: &str) -> Self {
        self.agixt_url = url.to_string();
        self
    }

    /// Create config with custom LocalAI URL
    pub fn with_localai_url(mut self, url: &str) -> Self {
        self.localai_url = url.to_string();
        self
    }

    /// Enable debug mode
    pub fn with_debug(mut self, debug: bool) -> Self {
        self.debug = debug;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = BridgeConfig::default();
        assert!(config.agixt_url.contains("7437"));
        assert!(config.localai_url.contains("8080"));
    }

    #[test]
    fn test_builder_pattern() {
        let config = BridgeConfig::default()
            .with_agixt_url("http://custom:7437")
            .with_debug(true);

        assert_eq!(config.agixt_url, "http://custom:7437");
        assert!(config.debug);
    }
}
