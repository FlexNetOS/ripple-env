use hdi::prelude::*;

/// Proposal entry type
/// Represents a decision to be made by the network
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Proposal {
    /// Unique identifier for the proposal
    pub id: String,
    /// Title of the proposal
    pub title: String,
    /// Detailed description of what is being proposed
    pub description: String,
    /// List of voting options
    pub options: Vec<String>,
    /// Deadline timestamp for voting
    pub deadline: Timestamp,
    /// Number of votes required to reach quorum (percentage 0-100)
    pub quorum_required: u8,
    /// Creator of the proposal
    pub creator: AgentPubKey,
    /// Timestamp when proposal was created
    pub created_at: Timestamp,
}

/// Vote entry type
/// Represents a single vote cast on a proposal
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Vote {
    /// Hash of the proposal being voted on
    pub proposal_hash: ActionHash,
    /// Public key of the voter
    pub voter_pubkey: AgentPubKey,
    /// Index of the chosen option (0-based)
    pub choice: u32,
    /// Voting weight (default 1, can be higher for weighted voting)
    pub weight: u32,
    /// Signature of the vote data for verification
    pub signature: Vec<u8>,
    /// Timestamp when vote was cast
    pub voted_at: Timestamp,
}

/// Decision entry type
/// Represents the finalized outcome of a proposal
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Decision {
    /// Hash of the proposal this decision is for
    pub proposal_hash: ActionHash,
    /// The winning option index (None if no quorum or tie)
    pub outcome: Option<u32>,
    /// Tally of votes for each option
    pub vote_tally: Vec<u32>,
    /// Total votes cast
    pub total_votes: u32,
    /// Whether quorum was reached
    pub quorum_reached: bool,
    /// Timestamp when decision was finalized
    pub finalized_at: Timestamp,
    /// Agent who finalized the decision
    pub finalized_by: AgentPubKey,
}

/// All entry types for the consensus zome
#[hdk_entry_defs]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_def(required_validations = 5)]
    Proposal(Proposal),
    #[entry_def(required_validations = 5)]
    Vote(Vote),
    #[entry_def(required_validations = 5)]
    Decision(Decision),
}

/// Link types for the consensus zome
#[hdk_link_types]
pub enum LinkTypes {
    /// Links from proposal to its votes
    ProposalToVotes,
    /// Links from agent to their votes
    AgentToVotes,
    /// Links from proposal to its decision
    ProposalToDecision,
    /// Anchor for all active proposals
    AllActiveProposals,
    /// Anchor for all finalized proposals
    AllFinalizedProposals,
    /// Links from agent to proposals they created
    AgentToProposals,
}

/// Validation rules for Proposal
pub fn validate_proposal(proposal: &Proposal) -> ExternResult<ValidateCallbackResult> {
    // ID must not be empty
    if proposal.id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Proposal ID cannot be empty".to_string(),
        ));
    }

    // Title must not be empty
    if proposal.title.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Proposal title cannot be empty".to_string(),
        ));
    }

    // Title length constraint
    if proposal.title.len() > 200 {
        return Ok(ValidateCallbackResult::Invalid(
            "Proposal title cannot exceed 200 characters".to_string(),
        ));
    }

    // Description must not be empty
    if proposal.description.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Proposal description cannot be empty".to_string(),
        ));
    }

    // Must have at least 2 options
    if proposal.options.len() < 2 {
        return Ok(ValidateCallbackResult::Invalid(
            "Proposal must have at least 2 voting options".to_string(),
        ));
    }

    // Options must not be empty
    for (i, option) in proposal.options.iter().enumerate() {
        if option.trim().is_empty() {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Voting option {} cannot be empty",
                i
            )));
        }
    }

    // Quorum must be between 1 and 100
    if proposal.quorum_required == 0 || proposal.quorum_required > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Quorum must be between 1 and 100 percent".to_string(),
        ));
    }

    // Deadline must be in the future (compared to creation time)
    if proposal.deadline <= proposal.created_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Proposal deadline must be after creation time".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validation rules for Vote
pub fn validate_vote(vote: &Vote) -> ExternResult<ValidateCallbackResult> {
    // Weight must be at least 1
    if vote.weight == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Vote weight must be at least 1".to_string(),
        ));
    }

    // Signature must not be empty
    if vote.signature.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Vote signature cannot be empty".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validation rules for Decision
pub fn validate_decision(decision: &Decision) -> ExternResult<ValidateCallbackResult> {
    // Vote tally must not be empty
    if decision.vote_tally.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Decision vote tally cannot be empty".to_string(),
        ));
    }

    // If outcome is set, it must be a valid index
    if let Some(outcome_idx) = decision.outcome {
        if outcome_idx as usize >= decision.vote_tally.len() {
            return Ok(ValidateCallbackResult::Invalid(
                "Decision outcome index is out of bounds".to_string(),
            ));
        }
    }

    // Total votes should match sum of tally
    let tally_sum: u32 = decision.vote_tally.iter().sum();
    if tally_sum != decision.total_votes {
        return Ok(ValidateCallbackResult::Invalid(
            "Vote tally sum does not match total votes".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Check if an agent has already voted on a proposal
/// Returns the existing vote links for validation
pub fn check_existing_vote(
    proposal_hash: &ActionHash,
    voter: &AgentPubKey,
) -> ExternResult<bool> {
    // This would need access to get_links which is only in hdk, not hdi
    // In integrity zome, we validate structure; coordinator checks duplicates
    // For now, we note that duplicate vote checking happens in coordinator
    let _ = proposal_hash;
    let _ = voter;
    Ok(false)
}

/// Main validation callback
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, .. }) => match app_entry {
            EntryTypes::Proposal(proposal) => validate_proposal(&proposal),
            EntryTypes::Vote(vote) => validate_vote(&vote),
            EntryTypes::Decision(decision) => validate_decision(&decision),
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry {
            app_entry,
            original_action_hash,
            ..
        }) => {
            match app_entry {
                EntryTypes::Proposal(proposal) => {
                    // Proposals can be updated before deadline
                    let _ = must_get_action(original_action_hash)?;
                    validate_proposal(&proposal)
                }
                EntryTypes::Vote(_) => {
                    // Votes cannot be updated, only deleted and re-cast
                    Ok(ValidateCallbackResult::Invalid(
                        "Votes cannot be updated. Delete and re-cast instead.".to_string(),
                    ))
                }
                EntryTypes::Decision(_) => {
                    // Decisions are final and cannot be updated
                    Ok(ValidateCallbackResult::Invalid(
                        "Decisions are final and cannot be updated.".to_string(),
                    ))
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
                LinkTypes::ProposalToVotes
                | LinkTypes::AgentToVotes
                | LinkTypes::ProposalToDecision
                | LinkTypes::AllActiveProposals
                | LinkTypes::AllFinalizedProposals
                | LinkTypes::AgentToProposals => Ok(ValidateCallbackResult::Valid),
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
