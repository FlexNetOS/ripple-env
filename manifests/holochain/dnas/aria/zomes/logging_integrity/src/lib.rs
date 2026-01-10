use hdi::prelude::*;

/// Log level enumeration for categorizing log severity
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
}

impl LogLevel {
    /// Validate that a string represents a valid log level
    pub fn from_str(s: &str) -> Option<LogLevel> {
        match s.to_lowercase().as_str() {
            "debug" => Some(LogLevel::Debug),
            "info" => Some(LogLevel::Info),
            "warn" | "warning" => Some(LogLevel::Warn),
            "error" => Some(LogLevel::Error),
            _ => None,
        }
    }

    /// Convert log level to string representation
    pub fn as_str(&self) -> &'static str {
        match self {
            LogLevel::Debug => "debug",
            LogLevel::Info => "info",
            LogLevel::Warn => "warn",
            LogLevel::Error => "error",
        }
    }
}

/// Log entry representing a single log message in the DHT
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct LogEntry {
    /// Timestamp when the log was created
    pub timestamp: Timestamp,
    /// Severity level of the log
    pub level: String,
    /// Source of the log (e.g., "agent:identity", "zome:storage", "system:conductor")
    pub source: String,
    /// The log message content
    pub message: String,
    /// Optional metadata as JSON string (serde_json::Value serialized)
    pub metadata: Option<String>,
}

/// Log stream for organizing and managing related logs
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct LogStream {
    /// Unique name for this log stream
    pub name: String,
    /// Number of days to retain logs (0 = indefinite)
    pub retention_days: u32,
    /// Maximum number of entries in this stream (0 = unlimited)
    pub max_entries: u32,
    /// Timestamp when this stream was created
    pub created_at: Timestamp,
    /// Optional description of this log stream
    pub description: Option<String>,
}

/// Log filter for querying logs with specific criteria
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct LogFilter {
    /// Filter name for saved filters
    pub name: String,
    /// Optional log level filter
    pub level: Option<String>,
    /// Optional source pattern filter
    pub source_pattern: Option<String>,
    /// Optional start timestamp for time range
    pub start_time: Option<Timestamp>,
    /// Optional end timestamp for time range
    pub end_time: Option<Timestamp>,
    /// Optional message pattern (substring search)
    pub message_pattern: Option<String>,
    /// Agent who created this filter
    pub created_by: AgentPubKey,
}

/// All entry types for the logging zome
#[hdk_entry_defs]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_def(required_validations = 3)]
    LogEntry(LogEntry),
    #[entry_def(required_validations = 5)]
    LogStream(LogStream),
    #[entry_def(required_validations = 5)]
    LogFilter(LogFilter),
}

/// Link types for the logging zome
#[hdk_link_types]
pub enum LinkTypes {
    /// Links from log stream to its entries
    StreamToEntry,
    /// Links from source identifier to log entries
    SourceToEntry,
    /// Links from log level to entries
    LevelToEntry,
    /// Links from timestamp anchor (daily bucket) to entries
    TimeToEntry,
    /// Links from agent to their saved filters
    AgentToFilter,
    /// All log streams anchor
    AllStreams,
    /// All logs anchor (for global queries)
    AllLogs,
}

/// Validation rules for LogEntry
pub fn validate_log_entry(entry: &LogEntry) -> ExternResult<ValidateCallbackResult> {
    // Validate log level
    let valid_levels = vec!["debug", "info", "warn", "warning", "error"];
    if !valid_levels.contains(&entry.level.to_lowercase().as_str()) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid log level: '{}'. Must be one of: {}",
            entry.level,
            valid_levels.join(", ")
        )));
    }

    // Validate timestamp is not in the future
    // Allow a small tolerance (5 minutes) for clock skew
    let now = sys_time()?;
    let tolerance_micros: i64 = 5 * 60 * 1_000_000; // 5 minutes in microseconds
    let future_limit = Timestamp::from_micros(now.as_micros() + tolerance_micros);

    if entry.timestamp > future_limit {
        return Ok(ValidateCallbackResult::Invalid(
            "Log timestamp cannot be in the future".to_string(),
        ));
    }

    // Validate source is not empty
    if entry.source.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Log source cannot be empty".to_string(),
        ));
    }

    // Validate source length
    if entry.source.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Log source cannot exceed 256 characters".to_string(),
        ));
    }

    // Validate message is not empty
    if entry.message.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Log message cannot be empty".to_string(),
        ));
    }

    // Validate message length (max 64KB)
    if entry.message.len() > 65536 {
        return Ok(ValidateCallbackResult::Invalid(
            "Log message cannot exceed 65536 characters".to_string(),
        ));
    }

    // Validate metadata is valid JSON if present
    if let Some(ref metadata) = entry.metadata {
        if !metadata.is_empty() {
            // Try to parse as JSON to validate structure
            if serde_json::from_str::<serde_json::Value>(metadata).is_err() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Log metadata must be valid JSON".to_string(),
                ));
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validation rules for LogStream
pub fn validate_log_stream(stream: &LogStream) -> ExternResult<ValidateCallbackResult> {
    // Name must not be empty
    if stream.name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Stream name cannot be empty".to_string(),
        ));
    }

    // Name length constraints
    if stream.name.len() > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Stream name cannot exceed 100 characters".to_string(),
        ));
    }

    // Validate name format (alphanumeric + underscore + hyphen)
    if !stream
        .name
        .chars()
        .all(|c| c.is_alphanumeric() || c == '_' || c == '-')
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Stream name can only contain alphanumeric characters, underscores, and hyphens"
                .to_string(),
        ));
    }

    // Validate retention_days is reasonable (max 10 years)
    if stream.retention_days > 3650 {
        return Ok(ValidateCallbackResult::Invalid(
            "Retention period cannot exceed 3650 days (10 years)".to_string(),
        ));
    }

    // Validate max_entries is reasonable
    if stream.max_entries > 10_000_000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Max entries cannot exceed 10,000,000".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validation rules for LogFilter
pub fn validate_log_filter(filter: &LogFilter) -> ExternResult<ValidateCallbackResult> {
    // Name must not be empty
    if filter.name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Filter name cannot be empty".to_string(),
        ));
    }

    // Name length constraints
    if filter.name.len() > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Filter name cannot exceed 100 characters".to_string(),
        ));
    }

    // Validate log level if specified
    if let Some(ref level) = filter.level {
        let valid_levels = vec!["debug", "info", "warn", "warning", "error"];
        if !valid_levels.contains(&level.to_lowercase().as_str()) {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Invalid log level in filter: '{}'. Must be one of: {}",
                level,
                valid_levels.join(", ")
            )));
        }
    }

    // Validate time range if both are specified
    if let (Some(start), Some(end)) = (filter.start_time, filter.end_time) {
        if start > end {
            return Ok(ValidateCallbackResult::Invalid(
                "Filter start time cannot be after end time".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, .. }) => match app_entry {
            EntryTypes::LogEntry(entry) => validate_log_entry(&entry),
            EntryTypes::LogStream(stream) => validate_log_stream(&stream),
            EntryTypes::LogFilter(filter) => validate_log_filter(&filter),
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry {
            app_entry,
            original_action_hash,
            ..
        }) => {
            // For updates, ensure the original entry exists and validate the new entry
            match app_entry {
                EntryTypes::LogEntry(entry) => {
                    let _ = must_get_action(original_action_hash)?;
                    validate_log_entry(&entry)
                }
                EntryTypes::LogStream(stream) => {
                    let _ = must_get_action(original_action_hash)?;
                    validate_log_stream(&stream)
                }
                EntryTypes::LogFilter(filter) => {
                    let _ = must_get_action(original_action_hash)?;
                    validate_log_filter(&filter)
                }
            }
        }
        FlatOp::StoreEntry(OpEntry::DeleteEntry {
            original_action_hash,
            ..
        }) => {
            // Verify the entry to be deleted exists
            let _ = must_get_action(original_action_hash)?;
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => {
            // Validate link creation based on link type
            match link_type {
                LinkTypes::StreamToEntry
                | LinkTypes::SourceToEntry
                | LinkTypes::LevelToEntry
                | LinkTypes::TimeToEntry
                | LinkTypes::AgentToFilter
                | LinkTypes::AllStreams
                | LinkTypes::AllLogs => Ok(ValidateCallbackResult::Valid),
            }
        }
        FlatOp::RegisterDeleteLink {
            link_type: _,
            base_address: _,
            target_address: _,
            tag: _,
            original_action: _,
            action: _,
        } => {
            // Allow link deletion
            Ok(ValidateCallbackResult::Valid)
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}
