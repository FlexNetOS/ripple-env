use hdk::prelude::*;
use consensus_integrity::*;

/// Anchor for all active proposals
const ACTIVE_PROPOSALS_ANCHOR: &str = "active_proposals";
/// Anchor for all finalized proposals
const FINALIZED_PROPOSALS_ANCHOR: &str = "finalized_proposals";

/// Input for creating a new proposal
#[derive(Serialize, Deserialize, Debug)]
pub struct CreateProposalInput {
    pub id: String,
    pub title: String,
    pub description: String,
    pub options: Vec<String>,
    pub deadline: Timestamp,
    pub quorum_required: u8,
}

/// Create a new proposal
#[hdk_extern]
pub fn create_proposal(input: CreateProposalInput) -> ExternResult<ActionHash> {
    let agent_info = agent_info()?;
    let now = sys_time()?;

    let proposal = Proposal {
        id: input.id,
        title: input.title,
        description: input.description,
        options: input.options,
        deadline: input.deadline,
        quorum_required: input.quorum_required,
        creator: agent_info.agent_latest_pubkey.clone(),
        created_at: now,
    };

    // Create the proposal entry
    let action_hash = create_entry(EntryTypes::Proposal(proposal.clone()))?;

    // Get the entry hash for linking
    let entry_hash = hash_entry(&proposal)?;

    // Link to active proposals anchor
    let anchor_hash = hash_entry(&ACTIVE_PROPOSALS_ANCHOR)?;
    create_link(
        anchor_hash,
        entry_hash.clone(),
        LinkTypes::AllActiveProposals,
        (),
    )?;

    // Link from creator to proposal
    create_link(
        agent_info.agent_latest_pubkey,
        entry_hash,
        LinkTypes::AgentToProposals,
        (),
    )?;

    Ok(action_hash)
}

/// Get a proposal by its action hash
#[hdk_extern]
pub fn get_proposal(action_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(action_hash, GetOptions::default())
}

/// List all active (non-finalized) proposals
#[hdk_extern]
pub fn list_active_proposals(_: ()) -> ExternResult<Vec<Record>> {
    let anchor_hash = hash_entry(&ACTIVE_PROPOSALS_ANCHOR)?;

    let links = get_links(
        GetLinksInputBuilder::try_new(anchor_hash, LinkTypes::AllActiveProposals)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// List all finalized proposals
#[hdk_extern]
pub fn list_finalized_proposals(_: ()) -> ExternResult<Vec<Record>> {
    let anchor_hash = hash_entry(&FINALIZED_PROPOSALS_ANCHOR)?;

    let links = get_links(
        GetLinksInputBuilder::try_new(anchor_hash, LinkTypes::AllFinalizedProposals)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Get proposals created by a specific agent
#[hdk_extern]
pub fn get_proposals_by_agent(agent: AgentPubKey) -> ExternResult<Vec<Record>> {
    let links = get_links(
        GetLinksInputBuilder::try_new(agent, LinkTypes::AgentToProposals)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Input for casting a vote
#[derive(Serialize, Deserialize, Debug)]
pub struct CastVoteInput {
    pub proposal_hash: ActionHash,
    pub choice: u32,
    pub weight: Option<u32>,
}

/// Cast a vote on a proposal
#[hdk_extern]
pub fn cast_vote(input: CastVoteInput) -> ExternResult<ActionHash> {
    let agent_info = agent_info()?;
    let now = sys_time()?;

    // Check that the proposal exists
    let proposal_record = get(input.proposal_hash.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Proposal not found".to_string())))?;

    // Extract proposal to validate choice
    let proposal: Proposal = proposal_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Failed to deserialize proposal".to_string())))?;

    // Validate choice is within options range
    if input.choice as usize >= proposal.options.len() {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid choice {}. Proposal has {} options (0-{})",
            input.choice,
            proposal.options.len(),
            proposal.options.len() - 1
        ))));
    }

    // Check deadline hasn't passed
    if now > proposal.deadline {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Voting deadline has passed".to_string()
        )));
    }

    // Check if agent has already voted
    let existing_votes = get_votes_by_agent_for_proposal(GetVotesByAgentInput {
        proposal_hash: input.proposal_hash.clone(),
        agent: agent_info.agent_latest_pubkey.clone(),
    })?;

    if !existing_votes.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Agent has already voted on this proposal. Delete existing vote first.".to_string()
        )));
    }

    // Create signature (simplified - in production, use proper cryptographic signing)
    let signature_data = format!(
        "{}:{}:{}:{}",
        input.proposal_hash,
        agent_info.agent_latest_pubkey,
        input.choice,
        now.as_micros()
    );
    let signature = signature_data.into_bytes();

    let vote = Vote {
        proposal_hash: input.proposal_hash.clone(),
        voter_pubkey: agent_info.agent_latest_pubkey.clone(),
        choice: input.choice,
        weight: input.weight.unwrap_or(1),
        signature,
        voted_at: now,
    };

    // Create the vote entry
    let action_hash = create_entry(EntryTypes::Vote(vote.clone()))?;

    // Get the entry hash for linking
    let entry_hash = hash_entry(&vote)?;

    // Link from proposal to vote
    create_link(
        input.proposal_hash,
        entry_hash.clone(),
        LinkTypes::ProposalToVotes,
        (),
    )?;

    // Link from agent to vote
    create_link(
        agent_info.agent_latest_pubkey,
        entry_hash,
        LinkTypes::AgentToVotes,
        (),
    )?;

    Ok(action_hash)
}

/// Get all votes for a proposal
#[hdk_extern]
pub fn get_votes_for_proposal(proposal_hash: ActionHash) -> ExternResult<Vec<Record>> {
    let links = get_links(
        GetLinksInputBuilder::try_new(proposal_hash, LinkTypes::ProposalToVotes)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Input for getting votes by agent for a specific proposal
#[derive(Serialize, Deserialize, Debug)]
pub struct GetVotesByAgentInput {
    pub proposal_hash: ActionHash,
    pub agent: AgentPubKey,
}

/// Get votes by a specific agent for a specific proposal
#[hdk_extern]
pub fn get_votes_by_agent_for_proposal(input: GetVotesByAgentInput) -> ExternResult<Vec<Record>> {
    let links = get_links(
        GetLinksInputBuilder::try_new(input.agent, LinkTypes::AgentToVotes)?.build(),
    )?;

    let mut matching_votes = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            if let Some(vote) = record.entry().to_app_option::<Vote>().ok().flatten() {
                if vote.proposal_hash == input.proposal_hash {
                    matching_votes.push(record);
                }
            }
        }
    }

    Ok(matching_votes)
}

/// Get all votes cast by the current agent
#[hdk_extern]
pub fn get_my_votes(_: ()) -> ExternResult<Vec<Record>> {
    let agent_pubkey = agent_info()?.agent_latest_pubkey;

    let links = get_links(
        GetLinksInputBuilder::try_new(agent_pubkey, LinkTypes::AgentToVotes)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Delete a vote (allows re-voting)
#[hdk_extern]
pub fn delete_vote(action_hash: ActionHash) -> ExternResult<ActionHash> {
    // Verify the vote exists and belongs to the current agent
    let vote_record = get(action_hash.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Vote not found".to_string())))?;

    let vote: Vote = vote_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Failed to deserialize vote".to_string())))?;

    let agent_pubkey = agent_info()?.agent_latest_pubkey;
    if vote.voter_pubkey != agent_pubkey {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Can only delete your own votes".to_string()
        )));
    }

    // Check if proposal is still active (not finalized)
    let proposal_record = get(vote.proposal_hash.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Proposal not found".to_string())))?;

    let proposal: Proposal = proposal_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Failed to deserialize proposal".to_string())))?;

    let now = sys_time()?;
    if now > proposal.deadline {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Cannot delete vote after deadline has passed".to_string()
        )));
    }

    delete_entry(action_hash)
}

/// Check voting power for the current agent
/// Returns the base voting weight (can be extended with delegation logic)
#[hdk_extern]
pub fn check_voting_power(_: ()) -> ExternResult<u32> {
    // Base voting power is 1
    // This can be extended to implement delegation, stake-based voting, etc.
    Ok(1)
}

/// Finalize a decision when quorum is met or deadline has passed
#[hdk_extern]
pub fn finalize_decision(proposal_hash: ActionHash) -> ExternResult<ActionHash> {
    let agent_info = agent_info()?;
    let now = sys_time()?;

    // Get the proposal
    let proposal_record = get(proposal_hash.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Proposal not found".to_string())))?;

    let proposal: Proposal = proposal_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Failed to deserialize proposal".to_string())))?;

    // Check if already finalized
    let decision_links = get_links(
        GetLinksInputBuilder::try_new(proposal_hash.clone(), LinkTypes::ProposalToDecision)?.build(),
    )?;

    if !decision_links.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal has already been finalized".to_string()
        )));
    }

    // Get all votes
    let vote_records = get_votes_for_proposal(proposal_hash.clone())?;

    // Calculate vote tally
    let num_options = proposal.options.len();
    let mut vote_tally: Vec<u32> = vec![0; num_options];
    let mut total_votes: u32 = 0;

    for record in &vote_records {
        if let Some(vote) = record.entry().to_app_option::<Vote>().ok().flatten() {
            if (vote.choice as usize) < num_options {
                vote_tally[vote.choice as usize] += vote.weight;
                total_votes += vote.weight;
            }
        }
    }

    // Determine if quorum is reached
    // Note: In a real system, you'd track total eligible voters
    // For simplicity, we use number of votes vs quorum threshold
    let quorum_reached = total_votes >= proposal.quorum_required as u32;

    // Check if we can finalize (deadline passed OR quorum reached)
    let deadline_passed = now > proposal.deadline;
    if !deadline_passed && !quorum_reached {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Cannot finalize: deadline not passed and quorum not reached".to_string()
        )));
    }

    // Determine outcome (highest vote count wins)
    let outcome = if quorum_reached && total_votes > 0 {
        let max_votes = vote_tally.iter().max().unwrap_or(&0);
        let winners: Vec<usize> = vote_tally
            .iter()
            .enumerate()
            .filter(|(_, v)| *v == max_votes)
            .map(|(i, _)| i)
            .collect();

        // Only set outcome if there's a clear winner (no tie)
        if winners.len() == 1 {
            Some(winners[0] as u32)
        } else {
            None // Tie
        }
    } else {
        None // No quorum or no votes
    };

    let decision = Decision {
        proposal_hash: proposal_hash.clone(),
        outcome,
        vote_tally,
        total_votes,
        quorum_reached,
        finalized_at: now,
        finalized_by: agent_info.agent_latest_pubkey,
    };

    // Create the decision entry
    let action_hash = create_entry(EntryTypes::Decision(decision.clone()))?;

    // Get entry hash for linking
    let entry_hash = hash_entry(&decision)?;

    // Link from proposal to decision
    create_link(
        proposal_hash.clone(),
        entry_hash.clone(),
        LinkTypes::ProposalToDecision,
        (),
    )?;

    // Move proposal from active to finalized
    let active_anchor_hash = hash_entry(&ACTIVE_PROPOSALS_ANCHOR)?;
    let finalized_anchor_hash = hash_entry(&FINALIZED_PROPOSALS_ANCHOR)?;
    let proposal_entry_hash = hash_entry(&proposal)?;

    // Remove from active proposals (delete the link)
    let active_links = get_links(
        GetLinksInputBuilder::try_new(active_anchor_hash, LinkTypes::AllActiveProposals)?.build(),
    )?;

    for link in active_links {
        if link.target == proposal_entry_hash.clone().into() {
            delete_link(link.create_link_hash)?;
        }
    }

    // Add to finalized proposals
    create_link(
        finalized_anchor_hash,
        proposal_entry_hash,
        LinkTypes::AllFinalizedProposals,
        (),
    )?;

    Ok(action_hash)
}

/// Get the decision for a proposal
#[hdk_extern]
pub fn get_decision_for_proposal(proposal_hash: ActionHash) -> ExternResult<Option<Record>> {
    let links = get_links(
        GetLinksInputBuilder::try_new(proposal_hash, LinkTypes::ProposalToDecision)?.build(),
    )?;

    if links.is_empty() {
        return Ok(None);
    }

    get(links[0].target.clone(), GetOptions::default())
}

/// Get proposal status (active, finalized, or expired)
#[derive(Serialize, Deserialize, Debug)]
pub struct ProposalStatus {
    pub proposal_hash: ActionHash,
    pub is_active: bool,
    pub is_finalized: bool,
    pub deadline_passed: bool,
    pub votes_count: u32,
    pub quorum_required: u8,
}

#[hdk_extern]
pub fn get_proposal_status(proposal_hash: ActionHash) -> ExternResult<ProposalStatus> {
    let now = sys_time()?;

    // Get the proposal
    let proposal_record = get(proposal_hash.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Proposal not found".to_string())))?;

    let proposal: Proposal = proposal_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Failed to deserialize proposal".to_string())))?;

    // Check if finalized
    let decision_links = get_links(
        GetLinksInputBuilder::try_new(proposal_hash.clone(), LinkTypes::ProposalToDecision)?.build(),
    )?;
    let is_finalized = !decision_links.is_empty();

    // Get vote count
    let votes = get_votes_for_proposal(proposal_hash.clone())?;
    let votes_count: u32 = votes
        .iter()
        .filter_map(|r| r.entry().to_app_option::<Vote>().ok().flatten())
        .map(|v| v.weight)
        .sum();

    let deadline_passed = now > proposal.deadline;

    Ok(ProposalStatus {
        proposal_hash,
        is_active: !is_finalized && !deadline_passed,
        is_finalized,
        deadline_passed,
        votes_count,
        quorum_required: proposal.quorum_required,
    })
}

/// Get agent info (convenience function)
#[hdk_extern]
pub fn whoami(_: ()) -> ExternResult<AgentInfo> {
    agent_info()
}
