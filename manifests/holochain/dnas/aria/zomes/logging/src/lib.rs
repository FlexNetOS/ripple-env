use hdk::prelude::*;
use logging_integrity::*;

/// Anchor for all log streams
const ALL_STREAMS_ANCHOR: &str = "all_log_streams";

/// Anchor for all logs (global)
const ALL_LOGS_ANCHOR: &str = "all_logs";

/// Input for writing a log entry
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WriteLogInput {
    pub level: String,
    pub source: String,
    pub message: String,
    pub metadata: Option<String>,
    pub stream_name: Option<String>,
}

/// Input for querying logs by time range
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TimeRangeInput {
    pub start: Timestamp,
    pub end: Timestamp,
    pub limit: Option<u32>,
}

/// Input for searching logs with filters
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SearchLogsInput {
    pub level: Option<String>,
    pub source: Option<String>,
    pub start_time: Option<Timestamp>,
    pub end_time: Option<Timestamp>,
    pub message_pattern: Option<String>,
    pub stream_name: Option<String>,
    pub limit: Option<u32>,
}

/// Helper to create an anchor hash from a string
fn anchor_hash(anchor: &str) -> ExternResult<EntryHash> {
    hash_entry(anchor)
}

/// Helper to create a level anchor (e.g., "level:info")
fn level_anchor(level: &str) -> String {
    format!("level:{}", level.to_lowercase())
}

/// Helper to create a source anchor
fn source_anchor(source: &str) -> String {
    format!("source:{}", source)
}

/// Helper to create a daily time bucket anchor
fn time_bucket_anchor(timestamp: Timestamp) -> String {
    // Create daily buckets: YYYY-MM-DD
    let micros = timestamp.as_micros();
    let seconds = micros / 1_000_000;
    let days_since_epoch = seconds / 86400;
    format!("time:{}", days_since_epoch)
}

/// Write a new log entry to the DHT
#[hdk_extern]
pub fn write_log(input: WriteLogInput) -> ExternResult<ActionHash> {
    let timestamp = sys_time()?;

    let log_entry = LogEntry {
        timestamp,
        level: input.level.clone(),
        source: input.source.clone(),
        message: input.message,
        metadata: input.metadata,
    };

    // Create the log entry
    let action_hash = create_entry(EntryTypes::LogEntry(log_entry.clone()))?;
    let entry_hash = hash_entry(&log_entry)?;

    // Link to global all_logs anchor
    let all_logs_hash = anchor_hash(ALL_LOGS_ANCHOR)?;
    create_link(
        all_logs_hash,
        entry_hash.clone(),
        LinkTypes::AllLogs,
        (),
    )?;

    // Link from level anchor
    let level_hash = anchor_hash(&level_anchor(&input.level))?;
    create_link(
        level_hash,
        entry_hash.clone(),
        LinkTypes::LevelToEntry,
        (),
    )?;

    // Link from source anchor
    let source_hash = anchor_hash(&source_anchor(&input.source))?;
    create_link(
        source_hash,
        entry_hash.clone(),
        LinkTypes::SourceToEntry,
        (),
    )?;

    // Link from time bucket anchor
    let time_hash = anchor_hash(&time_bucket_anchor(timestamp))?;
    create_link(
        time_hash,
        entry_hash.clone(),
        LinkTypes::TimeToEntry,
        (),
    )?;

    // If stream name specified, link to stream
    if let Some(stream_name) = input.stream_name {
        let stream_hash = anchor_hash(&format!("stream:{}", stream_name))?;
        create_link(
            stream_hash,
            entry_hash,
            LinkTypes::StreamToEntry,
            (),
        )?;
    }

    Ok(action_hash)
}

/// Get logs within a time range
#[hdk_extern]
pub fn get_logs_by_timerange(input: TimeRangeInput) -> ExternResult<Vec<Record>> {
    let start_micros = input.start.as_micros();
    let end_micros = input.end.as_micros();
    let start_days = start_micros / 1_000_000 / 86400;
    let end_days = end_micros / 1_000_000 / 86400;

    let mut all_records = Vec::new();
    let limit = input.limit.unwrap_or(1000) as usize;

    // Iterate through each day bucket
    for day in start_days..=end_days {
        let bucket_anchor = format!("time:{}", day);
        let bucket_hash = anchor_hash(&bucket_anchor)?;

        let links = get_links(
            GetLinksInputBuilder::try_new(bucket_hash, LinkTypes::TimeToEntry)?.build(),
        )?;

        for link in links {
            if let Some(record) = get(link.target, GetOptions::default())? {
                // Filter by exact time range
                if let Some(entry) = record.entry().as_option() {
                    if let Ok(log_entry) = LogEntry::try_from(entry.clone()) {
                        if log_entry.timestamp >= input.start && log_entry.timestamp <= input.end {
                            all_records.push(record);
                            if all_records.len() >= limit {
                                return Ok(all_records);
                            }
                        }
                    }
                }
            }
        }
    }

    // Sort by timestamp
    all_records.sort_by(|a, b| {
        let ts_a = a.entry().as_option().and_then(|e| {
            LogEntry::try_from(e.clone()).ok().map(|l| l.timestamp)
        });
        let ts_b = b.entry().as_option().and_then(|e| {
            LogEntry::try_from(e.clone()).ok().map(|l| l.timestamp)
        });
        ts_a.cmp(&ts_b)
    });

    Ok(all_records)
}

/// Get logs by severity level
#[hdk_extern]
pub fn get_logs_by_level(level: String) -> ExternResult<Vec<Record>> {
    let level_hash = anchor_hash(&level_anchor(&level))?;

    let links = get_links(
        GetLinksInputBuilder::try_new(level_hash, LinkTypes::LevelToEntry)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Get logs by source
#[hdk_extern]
pub fn get_logs_by_source(source: String) -> ExternResult<Vec<Record>> {
    let source_hash = anchor_hash(&source_anchor(&source))?;

    let links = get_links(
        GetLinksInputBuilder::try_new(source_hash, LinkTypes::SourceToEntry)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Create a new log stream
#[hdk_extern]
pub fn create_log_stream(stream: LogStream) -> ExternResult<ActionHash> {
    // Create the stream entry
    let action_hash = create_entry(EntryTypes::LogStream(stream.clone()))?;
    let entry_hash = hash_entry(&stream)?;

    // Link from all streams anchor
    let anchor_hash = anchor_hash(ALL_STREAMS_ANCHOR)?;
    create_link(
        anchor_hash,
        entry_hash,
        LinkTypes::AllStreams,
        (),
    )?;

    Ok(action_hash)
}

/// Get all log streams
#[hdk_extern]
pub fn get_all_streams(_: ()) -> ExternResult<Vec<Record>> {
    let anchor_hash = anchor_hash(ALL_STREAMS_ANCHOR)?;

    let links = get_links(
        GetLinksInputBuilder::try_new(anchor_hash, LinkTypes::AllStreams)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Get logs for a specific stream
#[hdk_extern]
pub fn get_stream_logs(stream_name: String) -> ExternResult<Vec<Record>> {
    let stream_hash = anchor_hash(&format!("stream:{}", stream_name))?;

    let links = get_links(
        GetLinksInputBuilder::try_new(stream_hash, LinkTypes::StreamToEntry)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    // Sort by timestamp (newest first)
    records.sort_by(|a, b| {
        let ts_a = a.entry().as_option().and_then(|e| {
            LogEntry::try_from(e.clone()).ok().map(|l| l.timestamp)
        });
        let ts_b = b.entry().as_option().and_then(|e| {
            LogEntry::try_from(e.clone()).ok().map(|l| l.timestamp)
        });
        ts_b.cmp(&ts_a) // Reverse order for newest first
    });

    Ok(records)
}

/// Search logs with comprehensive filter support
#[hdk_extern]
pub fn search_logs(input: SearchLogsInput) -> ExternResult<Vec<Record>> {
    let limit = input.limit.unwrap_or(100) as usize;
    let mut results = Vec::new();

    // Determine the best starting point based on filters
    let candidate_records: Vec<Record> = if let Some(ref level) = input.level {
        // If level filter specified, start with level-indexed logs
        get_logs_by_level(level.clone())?
    } else if let Some(ref source) = input.source {
        // If source filter specified, start with source-indexed logs
        get_logs_by_source(source.clone())?
    } else if let (Some(start), Some(end)) = (input.start_time, input.end_time) {
        // If time range specified, use time-indexed logs
        get_logs_by_timerange(TimeRangeInput {
            start,
            end,
            limit: Some(limit as u32 * 2), // Get more to allow for filtering
        })?
    } else {
        // Default: get all logs
        let all_logs_hash = anchor_hash(ALL_LOGS_ANCHOR)?;
        let links = get_links(
            GetLinksInputBuilder::try_new(all_logs_hash, LinkTypes::AllLogs)?.build(),
        )?;

        let mut records = Vec::new();
        for link in links {
            if let Some(record) = get(link.target, GetOptions::default())? {
                records.push(record);
            }
        }
        records
    };

    // Apply all filters
    for record in candidate_records {
        if results.len() >= limit {
            break;
        }

        if let Some(entry) = record.entry().as_option() {
            if let Ok(log_entry) = LogEntry::try_from(entry.clone()) {
                // Apply level filter (if not already filtered by level query)
                if let Some(ref level) = input.level {
                    if log_entry.level.to_lowercase() != level.to_lowercase() {
                        continue;
                    }
                }

                // Apply source filter
                if let Some(ref source) = input.source {
                    if !log_entry.source.contains(source) {
                        continue;
                    }
                }

                // Apply time range filter
                if let Some(start) = input.start_time {
                    if log_entry.timestamp < start {
                        continue;
                    }
                }
                if let Some(end) = input.end_time {
                    if log_entry.timestamp > end {
                        continue;
                    }
                }

                // Apply message pattern filter
                if let Some(ref pattern) = input.message_pattern {
                    if !log_entry.message.to_lowercase().contains(&pattern.to_lowercase()) {
                        continue;
                    }
                }

                // Apply stream filter if specified
                // Note: This requires checking if the log is linked to the stream
                // For now, we include logs that pass other filters

                results.push(record);
            }
        }
    }

    // Sort by timestamp (newest first)
    results.sort_by(|a, b| {
        let ts_a = a.entry().as_option().and_then(|e| {
            LogEntry::try_from(e.clone()).ok().map(|l| l.timestamp)
        });
        let ts_b = b.entry().as_option().and_then(|e| {
            LogEntry::try_from(e.clone()).ok().map(|l| l.timestamp)
        });
        ts_b.cmp(&ts_a)
    });

    Ok(results)
}

/// Save a log filter for later use
#[hdk_extern]
pub fn save_filter(filter: LogFilter) -> ExternResult<ActionHash> {
    let action_hash = create_entry(EntryTypes::LogFilter(filter.clone()))?;
    let entry_hash = hash_entry(&filter)?;

    // Link from agent to filter
    let agent_pubkey = agent_info()?.agent_latest_pubkey;
    create_link(
        agent_pubkey,
        entry_hash,
        LinkTypes::AgentToFilter,
        (),
    )?;

    Ok(action_hash)
}

/// Get all saved filters for the current agent
#[hdk_extern]
pub fn get_my_filters(_: ()) -> ExternResult<Vec<Record>> {
    let agent_pubkey = agent_info()?.agent_latest_pubkey;

    let links = get_links(
        GetLinksInputBuilder::try_new(agent_pubkey, LinkTypes::AgentToFilter)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Delete a saved filter
#[hdk_extern]
pub fn delete_filter(action_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(action_hash)
}

/// Get log entry by action hash
#[hdk_extern]
pub fn get_log_entry(action_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(action_hash, GetOptions::default())
}

/// Delete a log entry
#[hdk_extern]
pub fn delete_log_entry(action_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(action_hash)
}

/// Get agent info (convenience wrapper)
#[hdk_extern]
pub fn whoami(_: ()) -> ExternResult<AgentInfo> {
    agent_info()
}

/// Get current system time (convenience wrapper)
#[hdk_extern]
pub fn now(_: ()) -> ExternResult<Timestamp> {
    sys_time()
}
