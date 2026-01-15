// React Query hooks for workspace operations
import { useQuery, useMutation, useQueryClient } from '@tanstack/react-query';
import { workspaceApi, processApi } from '@/lib/vibe-kanban/client';
import type { 
  Workspace, 
  WorkspaceWithStatus, 
  ExecutionProcess,
  Diff,
  BranchStatus,
  CreateFollowUpAttempt,
} from '@/shared/vibe-kanban/types';

// Query keys
export const workspaceKeys = {
  all: ['workspaces'] as const,
  lists: () => [...workspaceKeys.all, 'list'] as const,
  list: (taskId: string) => [...workspaceKeys.lists(), taskId] as const,
  details: () => [...workspaceKeys.all, 'detail'] as const,
  detail: (id: string) => [...workspaceKeys.details(), id] as const,
  diffs: (id: string, repoId: string) => [...workspaceKeys.detail(id), 'diffs', repoId] as const,
  branchStatus: (id: string, repoId: string) => [...workspaceKeys.detail(id), 'branch-status', repoId] as const,
};

export const processKeys = {
  all: ['processes'] as const,
  lists: () => [...processKeys.all, 'list'] as const,
  list: (workspaceId: string) => [...processKeys.lists(), workspaceId] as const,
  details: () => [...processKeys.all, 'detail'] as const,
  detail: (id: string) => [...processKeys.details(), id] as const,
  logs: (id: string) => [...processKeys.detail(id), 'logs'] as const,
};

// List workspaces for a task
export function useWorkspaces(taskId: string | null) {
  return useQuery({
    queryKey: workspaceKeys.list(taskId || ''),
    queryFn: async () => {
      if (!taskId) return [];
      const { data } = await workspaceApi.list(taskId);
      return data;
    },
    enabled: !!taskId,
    refetchInterval: 10000, // Poll every 10 seconds for status updates
  });
}

// Get single workspace
export function useWorkspace(workspaceId: string | null) {
  return useQuery({
    queryKey: workspaceKeys.detail(workspaceId || ''),
    queryFn: async () => {
      if (!workspaceId) return null;
      const { data } = await workspaceApi.get(workspaceId);
      return data;
    },
    enabled: !!workspaceId,
    refetchInterval: 5000, // Poll for status updates
  });
}

// Get workspace diffs
export function useWorkspaceDiffs(workspaceId: string | null, repoId: string | null) {
  return useQuery({
    queryKey: workspaceKeys.diffs(workspaceId || '', repoId || ''),
    queryFn: async () => {
      if (!workspaceId || !repoId) return [];
      const { data } = await workspaceApi.getDiffs(workspaceId, repoId);
      return data;
    },
    enabled: !!workspaceId && !!repoId,
  });
}

// Get branch status
export function useBranchStatus(workspaceId: string | null, repoId: string | null) {
  return useQuery({
    queryKey: workspaceKeys.branchStatus(workspaceId || '', repoId || ''),
    queryFn: async () => {
      if (!workspaceId || !repoId) return null;
      const { data } = await workspaceApi.getBranchStatus(workspaceId, repoId);
      return data;
    },
    enabled: !!workspaceId && !!repoId,
  });
}

// Start workspace agent
export function useStartWorkspace() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async ({ workspaceId, executorProfileId }: { workspaceId: string; executorProfileId: string }) => {
      const { data } = await workspaceApi.start(workspaceId, executorProfileId);
      return data;
    },
    onSuccess: (_, { workspaceId }) => {
      queryClient.invalidateQueries({ queryKey: workspaceKeys.detail(workspaceId) });
    },
  });
}

// Stop workspace agent
export function useStopWorkspace() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async (workspaceId: string) => {
      const { data } = await workspaceApi.stop(workspaceId);
      return data;
    },
    onSuccess: (_, workspaceId) => {
      queryClient.invalidateQueries({ queryKey: workspaceKeys.detail(workspaceId) });
    },
  });
}

// Follow-up prompt
export function useFollowUp() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async ({ workspaceId, data }: { workspaceId: string; data: CreateFollowUpAttempt }) => {
      const { data: result } = await workspaceApi.followUp(workspaceId, data);
      return result;
    },
    onSuccess: (_, { workspaceId }) => {
      queryClient.invalidateQueries({ queryKey: workspaceKeys.detail(workspaceId) });
    },
  });
}

// Merge changes
export function useMergeWorkspace() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async ({ workspaceId, repoId }: { workspaceId: string; repoId: string }) => {
      const { data } = await workspaceApi.merge(workspaceId, repoId);
      return data;
    },
    onSuccess: (_, { workspaceId, repoId }) => {
      queryClient.invalidateQueries({ queryKey: workspaceKeys.diffs(workspaceId, repoId) });
      queryClient.invalidateQueries({ queryKey: workspaceKeys.branchStatus(workspaceId, repoId) });
    },
  });
}

// Push changes
export function usePushWorkspace() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async ({ workspaceId, repoId }: { workspaceId: string; repoId: string }) => {
      const { data } = await workspaceApi.push(workspaceId, repoId);
      return data;
    },
    onSuccess: (_, { workspaceId, repoId }) => {
      queryClient.invalidateQueries({ queryKey: workspaceKeys.branchStatus(workspaceId, repoId) });
    },
  });
}

// Process hooks
export function useProcesses(workspaceId: string | null) {
  return useQuery({
    queryKey: processKeys.list(workspaceId || ''),
    queryFn: async () => {
      if (!workspaceId) return [];
      const { data } = await processApi.listByWorkspace(workspaceId);
      return data;
    },
    enabled: !!workspaceId,
    refetchInterval: 5000,
  });
}

export function useProcessLogs(processId: string | null) {
  return useQuery({
    queryKey: processKeys.logs(processId || ''),
    queryFn: async () => {
      if (!processId) return [];
      const { data } = await processApi.getLogs(processId);
      return data;
    },
    enabled: !!processId,
    refetchInterval: 2000, // Poll for new logs
  });
}

export function useKillProcess() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async (processId: string) => {
      const { data } = await processApi.kill(processId);
      return data;
    },
    onSuccess: () => {
      queryClient.invalidateQueries({ queryKey: processKeys.all });
      queryClient.invalidateQueries({ queryKey: workspaceKeys.all });
    },
  });
}
