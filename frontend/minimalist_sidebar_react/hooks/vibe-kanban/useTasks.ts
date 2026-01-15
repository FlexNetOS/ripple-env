// React Query hooks for task operations
import { useQuery, useMutation, useQueryClient } from '@tanstack/react-query';
import { taskApi } from '@/lib/vibe-kanban/client';
import type { Task, TaskWithAttemptStatus, CreateTask, UpdateTask, TaskStatus } from '@/shared/vibe-kanban/types';

// Query keys
export const taskKeys = {
  all: ['tasks'] as const,
  lists: () => [...taskKeys.all, 'list'] as const,
  list: (projectId: string) => [...taskKeys.lists(), projectId] as const,
  details: () => [...taskKeys.all, 'detail'] as const,
  detail: (id: string) => [...taskKeys.details(), id] as const,
};

// List tasks for a project
export function useTasks(projectId: string | null) {
  return useQuery({
    queryKey: taskKeys.list(projectId || ''),
    queryFn: async () => {
      if (!projectId) return [];
      const { data } = await taskApi.list(projectId);
      return data;
    },
    enabled: !!projectId,
    staleTime: 30000, // 30 seconds
  });
}

// Get single task
export function useTask(taskId: string | null) {
  return useQuery({
    queryKey: taskKeys.detail(taskId || ''),
    queryFn: async () => {
      if (!taskId) return null;
      const { data } = await taskApi.get(taskId);
      return data;
    },
    enabled: !!taskId,
  });
}

// Create task mutation
export function useCreateTask() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async (data: CreateTask) => {
      const { data: task } = await taskApi.create(data);
      return task;
    },
    onSuccess: (task) => {
      // Invalidate task list for the project
      queryClient.invalidateQueries({ queryKey: taskKeys.list(task.project_id) });
    },
  });
}

// Update task mutation
export function useUpdateTask() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async ({ taskId, data }: { taskId: string; data: UpdateTask }) => {
      const { data: task } = await taskApi.update(taskId, data);
      return task;
    },
    onSuccess: (task) => {
      queryClient.invalidateQueries({ queryKey: taskKeys.detail(task.id) });
      queryClient.invalidateQueries({ queryKey: taskKeys.list(task.project_id) });
    },
  });
}

// Update task status (commonly used)
export function useUpdateTaskStatus() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async ({ taskId, status }: { taskId: string; status: TaskStatus }) => {
      const { data: task } = await taskApi.update(taskId, { status });
      return task;
    },
    onMutate: async ({ taskId, status }) => {
      // Optimistic update
      await queryClient.cancelQueries({ queryKey: taskKeys.all });
      
      // Get current task data
      const previousTask = queryClient.getQueryData<Task>(taskKeys.detail(taskId));
      
      // Update optimistically
      if (previousTask) {
        queryClient.setQueryData(taskKeys.detail(taskId), {
          ...previousTask,
          status,
        });
      }
      
      return { previousTask };
    },
    onError: (err, variables, context) => {
      // Rollback on error
      if (context?.previousTask) {
        queryClient.setQueryData(
          taskKeys.detail(variables.taskId),
          context.previousTask
        );
      }
    },
    onSettled: (task) => {
      if (task) {
        queryClient.invalidateQueries({ queryKey: taskKeys.detail(task.id) });
        queryClient.invalidateQueries({ queryKey: taskKeys.list(task.project_id) });
      }
    },
  });
}

// Delete task mutation
export function useDeleteTask() {
  const queryClient = useQueryClient();
  
  return useMutation({
    mutationFn: async ({ taskId, projectId }: { taskId: string; projectId: string }) => {
      await taskApi.delete(taskId);
      return { taskId, projectId };
    },
    onSuccess: ({ projectId }) => {
      queryClient.invalidateQueries({ queryKey: taskKeys.list(projectId) });
    },
  });
}
