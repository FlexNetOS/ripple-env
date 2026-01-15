// React Query hooks for project operations
import { useQuery, useMutation, useQueryClient } from '@tanstack/react-query';
import { projectApi } from '@/lib/vibe-kanban/client';
import type { Project, CreateProject, Repo } from '@/shared/vibe-kanban/types';

// Query keys
export const projectKeys = {
  all: ['projects'] as const,
  lists: () => [...projectKeys.all, 'list'] as const,
  list: () => [...projectKeys.lists()] as const,
  details: () => [...projectKeys.all, 'detail'] as const,
  detail: (id: string) => [...projectKeys.details(), id] as const,
  repos: (id: string) => [...projectKeys.detail(id), 'repos'] as const,
};

// List all projects
export function useProjects() {
  return useQuery({
    queryKey: projectKeys.list(),
    queryFn: async () => {
      const { data } = await projectApi.list();
      return data;
    },
    staleTime: 60000, // 1 minute
  });
}

// Get single project
export function useProject(projectId: string | null) {
  return useQuery({
    queryKey: projectKeys.detail(projectId || ''),
    queryFn: async () => {
      if (!projectId) return null;
      const { data } = await projectApi.get(projectId);
      return data;
    },
    enabled: !!projectId,
  });
}

// Get project repos
export function useProjectRepos(projectId: string | null) {
  return useQuery({
    queryKey: projectKeys.repos(projectId || ''),
    queryFn: async () => {
      if (!projectId) return [];
      const { data } = await projectApi.getRepos(projectId);
      return data;
    },
    enabled: !!projectId,
  });
}

// Create project mutation
export function useCreateProject() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async (data: CreateProject) => {
      const { data: project } = await projectApi.create(data);
      return project;
    },
    onSuccess: () => {
      queryClient.invalidateQueries({ queryKey: projectKeys.list() });
    },
  });
}

// Update project mutation
export function useUpdateProject() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async ({ projectId, data }: { projectId: string; data: Partial<CreateProject> }) => {
      const { data: project } = await projectApi.update(projectId, data);
      return project;
    },
    onSuccess: (project) => {
      queryClient.invalidateQueries({ queryKey: projectKeys.detail(project.id) });
      queryClient.invalidateQueries({ queryKey: projectKeys.list() });
    },
  });
}

// Delete project mutation
export function useDeleteProject() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async (projectId: string) => {
      await projectApi.delete(projectId);
      return projectId;
    },
    onSuccess: () => {
      queryClient.invalidateQueries({ queryKey: projectKeys.list() });
    },
  });
}
