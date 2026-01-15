// Adapted types from vibe-kanban for minimalist_sidebar integration
// Converted Date -> string and bigint -> number for JSON serialization

export type TaskStatus = 'todo' | 'inprogress' | 'inreview' | 'done' | 'cancelled';

export interface Task {
  id: string;
  project_id: string;
  title: string;
  description: string | null;
  status: TaskStatus;
  parent_workspace_id: string | null;
  shared_task_id: string | null;
  created_at: string;
  updated_at: string;
}

export interface TaskWithAttemptStatus extends Task {
  has_in_progress_attempt: boolean;
  last_attempt_failed: boolean;
  executor: string;
}

export interface CreateTask {
  project_id: string;
  title: string;
  description: string | null;
  status: TaskStatus | null;
  parent_workspace_id: string | null;
  image_ids: string[] | null;
  shared_task_id: string | null;
}

export interface UpdateTask {
  title?: string | null;
  description?: string | null;
  status?: TaskStatus | null;
  parent_workspace_id?: string | null;
  image_ids?: string[] | null;
}

export interface Project {
  id: string;
  name: string;
  default_agent_working_dir: string | null;
  remote_project_id: string | null;
  created_at: string;
  updated_at: string;
}

export interface CreateProject {
  name: string;
  repositories: CreateProjectRepo[];
}

export interface CreateProjectRepo {
  display_name: string;
  git_repo_path: string;
}

export interface Repo {
  id: string;
  path: string;
  name: string;
  display_name: string;
  setup_script: string | null;
  cleanup_script: string | null;
  copy_files: string | null;
  parallel_setup_script: boolean;
  dev_server_script: string | null;
  created_at: string;
  updated_at: string;
}

export interface Workspace {
  id: string;
  task_id: string;
  container_ref: string | null;
  branch: string;
  agent_working_dir: string | null;
  setup_completed_at: string | null;
  created_at: string;
  updated_at: string;
  archived: boolean;
  pinned: boolean;
  name: string | null;
}

export interface WorkspaceWithStatus extends Workspace {
  is_running: boolean;
  is_errored: boolean;
}

export interface Session {
  id: string;
  workspace_id: string;
  executor: string | null;
  created_at: string;
  updated_at: string;
}

export type ExecutionProcessStatus = 'running' | 'completed' | 'failed' | 'killed';
export type ExecutionProcessRunReason = 'setupscript' | 'cleanupscript' | 'codingagent' | 'devserver';

export interface ExecutorAction {
  type: string;
  data?: unknown;
}

export interface ExecutionProcess {
  id: string;
  session_id: string;
  run_reason: ExecutionProcessRunReason;
  executor_action: ExecutorAction;
  status: ExecutionProcessStatus;
  exit_code: number | null;
  dropped: boolean;
  started_at: string;
  completed_at: string | null;
  created_at: string;
  updated_at: string;
}

export type ApprovalStatus = 
  | { status: 'pending' }
  | { status: 'approved' }
  | { status: 'denied'; reason?: string }
  | { status: 'timed_out' };

export interface ApprovalRequest {
  id: string;
  execution_process_id: string;
  tool_name: string;
  tool_input: unknown;
  tool_call_id: string;
  status: ApprovalStatus;
  created_at: string;
}

export type DiffChangeKind = 'added' | 'deleted' | 'modified' | 'renamed' | 'copied' | 'permissionChange';

export interface Diff {
  change: DiffChangeKind;
  oldPath: string | null;
  newPath: string | null;
  oldContent: string | null;
  newContent: string | null;
  contentOmitted: boolean;
  additions: number | null;
  deletions: number | null;
  repoId: string | null;
}

export interface BranchStatus {
  commits_behind: number | null;
  commits_ahead: number | null;
  has_uncommitted_changes: boolean | null;
  head_oid: string | null;
  uncommitted_count: number | null;
  untracked_count: number | null;
  target_branch_name: string;
  remote_commits_behind: number | null;
  remote_commits_ahead: number | null;
  is_rebase_in_progress: boolean;
  conflict_op: string | null;
  conflicted_files: string[];
}

export interface WorkspaceRepoInput {
  repo_id: string;
  target_branch: string;
}

export interface CreateTaskAttemptBody {
  task_id: string;
  executor_profile_id: string;
  repos: WorkspaceRepoInput[];
}

export interface CreateFollowUpAttempt {
  prompt: string;
  variant: string | null;
  retry_process_id: string | null;
  force_when_dirty: boolean | null;
  perform_git_reset: boolean | null;
}

// Agent types
export type BaseCodingAgent = 'CLAUDE_CODE' | 'CODEX' | 'GEMINI_CLI' | 'CUSTOM';

export interface ExecutorConfig {
  name: string;
  description: string;
  available: boolean;
}

export interface UserSystemInfo {
  login_status: LoginStatus;
  capabilities: Record<string, string[]>;
  executors: Record<BaseCodingAgent, ExecutorConfig>;
}

export type LoginStatus = 
  | { status: 'loggedout' }
  | { status: 'loggedin'; profile: ProfileResponse };

export interface ProfileResponse {
  user_id: string;
  username: string | null;
  email: string;
  providers: ProviderProfile[];
}

export interface ProviderProfile {
  provider: string;
  username: string | null;
  display_name: string | null;
  email: string | null;
  avatar_url: string | null;
}

// API Response wrapper
export interface ApiResponse<T, E = T> {
  success: boolean;
  data: T | null;
  error_data: E | null;
  message: string | null;
}
