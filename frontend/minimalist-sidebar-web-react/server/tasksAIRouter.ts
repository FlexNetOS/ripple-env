import { protectedProcedure, router } from "./_core/trpc";
import { z } from "zod";
import * as db from "./db";
import { OpenAI } from "openai";

// Initialize OpenAI client (uses environment variables automatically)
const openai = new OpenAI();

export const tasksAIRouter = router({
  // ==================== AI TASK SUGGESTIONS ====================

  // Generate task suggestions based on user's existing tasks and patterns
  generateSuggestions: protectedProcedure
    .input(z.object({
      context: z.string().optional(), // Optional context like "work", "personal", "project"
      count: z.number().min(1).max(10).default(5),
    }))
    .mutation(async ({ input, ctx }) => {
      const tasks = await db.getTasksByUserId(ctx.user.id);

      // Analyze existing tasks to understand patterns
      const taskTitles = tasks.map(t => t.title).join(", ");
      const taskDescriptions = tasks.map(t => t.description).filter(Boolean).join(". ");
      const commonTags = tasks.flatMap(t => t.tags || []);
      const tagFrequency = commonTags.reduce((acc, tag) => {
        acc[tag] = (acc[tag] || 0) + 1;
        return acc;
      }, {} as Record<string, number>);
      const topTags = Object.entries(tagFrequency)
        .sort(([, a], [, b]) => b - a)
        .slice(0, 5)
        .map(([tag]) => tag);

      const prompt = `You are an AI task management assistant. Based on the user's existing tasks, suggest ${input.count} new tasks that would be helpful.

Context: ${input.context || "general productivity"}

Existing tasks: ${taskTitles}
Common tags: ${topTags.join(", ")}

Generate ${input.count} task suggestions in JSON format:
[
  {
    "title": "Task title",
    "description": "Brief description",
    "priority": "low|medium|high|urgent",
    "tags": ["tag1", "tag2"],
    "estimatedTime": 60,
    "reasoning": "Why this task is suggested"
  }
]

Focus on:
1. Complementary tasks that support existing work
2. Maintenance and follow-up tasks
3. Strategic planning tasks
4. Learning and improvement tasks

Return only valid JSON array, no additional text.`;

      try {
        const response = await openai.chat.completions.create({
          model: "gpt-4.1-mini",
          messages: [{ role: "user", content: prompt }],
          temperature: 0.7,
          max_tokens: 1000,
        });

        const content = response.choices[0].message.content || "[]";
        const suggestions = JSON.parse(content);

        return { suggestions };
      } catch (error) {
        console.error("AI suggestion error:", error);
        // Fallback to rule-based suggestions
        return {
          suggestions: [
            {
              title: "Review and prioritize tasks",
              description: "Take time to review your current tasks and adjust priorities",
              priority: "medium",
              tags: ["planning", "review"],
              estimatedTime: 30,
              reasoning: "Regular task review helps maintain focus",
            },
            {
              title: "Plan next week's goals",
              description: "Set clear objectives for the upcoming week",
              priority: "high",
              tags: ["planning", "goals"],
              estimatedTime: 45,
              reasoning: "Weekly planning improves productivity",
            },
          ],
        };
      }
    }),

  // ==================== SMART SUBTASK GENERATION ====================

  // Generate subtasks for a complex task using AI
  generateSubtasks: protectedProcedure
    .input(z.object({
      taskId: z.number(),
      count: z.number().min(2).max(10).default(5),
    }))
    .mutation(async ({ input, ctx }) => {
      const task = await db.getTaskById(input.taskId);

      if (!task || task.userId !== ctx.user.id) {
        throw new Error("Task not found");
      }

      const prompt = `You are an AI task management assistant. Break down this task into ${input.count} actionable subtasks.

Task: ${task.title}
Description: ${task.description || "No description"}
Priority: ${task.priority}
Estimated time: ${task.estimatedTime || "unknown"} minutes

Generate ${input.count} subtasks in JSON format:
[
  {
    "title": "Subtask title",
    "description": "What needs to be done",
    "estimatedTime": 30,
    "order": 1
  }
]

Guidelines:
1. Make subtasks specific and actionable
2. Order them logically (dependencies first)
3. Distribute time estimates realistically
4. Each subtask should be completable independently

Return only valid JSON array, no additional text.`;

      try {
        const response = await openai.chat.completions.create({
          model: "gpt-4.1-mini",
          messages: [{ role: "user", content: prompt }],
          temperature: 0.6,
          max_tokens: 800,
        });

        const content = response.choices[0].message.content || "[]";
        const subtasks = JSON.parse(content);

        // Create the subtasks in the database
        const createdSubtasks = await Promise.all(
          subtasks.map(async (subtask: any) => {
            return db.createTask({
              userId: ctx.user.id,
              title: subtask.title,
              description: subtask.description,
              status: "todo",
              priority: task.priority, // Inherit parent priority
              parentTaskId: task.id,
              estimatedTime: subtask.estimatedTime,
              tags: task.tags || [],
              progress: 0,
              starred: false,
              hasInProgressAttempt: false,
              lastAttemptFailed: false,
            });
          })
        );

        // Create activity
        await db.createTaskActivity({
          taskId: task.id,
          type: "updated",
          user: "AI Assistant",
          description: `generated ${createdSubtasks.length} subtasks`,
        });

        return { subtasks: createdSubtasks };
      } catch (error) {
        console.error("AI subtask generation error:", error);
        throw new Error("Failed to generate subtasks");
      }
    }),

  // ==================== PRIORITY PREDICTION ====================

  // Predict optimal priority for a task based on content and context
  predictPriority: protectedProcedure
    .input(z.object({
      title: z.string(),
      description: z.string().optional(),
      dueDate: z.date().optional(),
      tags: z.array(z.string()).optional(),
    }))
    .mutation(async ({ input, ctx }) => {
      const tasks = await db.getTasksByUserId(ctx.user.id);

      // Calculate days until due date
      const daysUntilDue = input.dueDate
        ? Math.ceil((new Date(input.dueDate).getTime() - Date.now()) / (1000 * 60 * 60 * 24))
        : null;

      const prompt = `You are an AI task priority advisor. Analyze this task and suggest the optimal priority level.

Task: ${input.title}
Description: ${input.description || "No description"}
Due date: ${input.dueDate ? `${daysUntilDue} days from now` : "No deadline"}
Tags: ${input.tags?.join(", ") || "None"}

User has ${tasks.length} existing tasks:
- ${tasks.filter(t => t.priority === "urgent").length} urgent
- ${tasks.filter(t => t.priority === "high").length} high
- ${tasks.filter(t => t.priority === "medium").length} medium
- ${tasks.filter(t => t.priority === "low").length} low

Respond in JSON format:
{
  "priority": "low|medium|high|urgent",
  "confidence": 0.0-1.0,
  "reasoning": "Explanation for the priority level",
  "recommendations": ["tip1", "tip2"]
}

Consider:
1. Urgency based on deadline
2. Impact and importance
3. Dependencies and blockers
4. User's current workload
5. Task complexity

Return only valid JSON, no additional text.`;

      try {
        const response = await openai.chat.completions.create({
          model: "gpt-4.1-mini",
          messages: [{ role: "user", content: prompt }],
          temperature: 0.5,
          max_tokens: 400,
        });

        const content = response.choices[0].message.content || "{}";
        const prediction = JSON.parse(content);

        return prediction;
      } catch (error) {
        console.error("AI priority prediction error:", error);
        // Fallback to rule-based prediction
        let priority: "low" | "medium" | "high" | "urgent" = "medium";

        if (daysUntilDue !== null) {
          if (daysUntilDue <= 1) priority = "urgent";
          else if (daysUntilDue <= 3) priority = "high";
          else if (daysUntilDue <= 7) priority = "medium";
          else priority = "low";
        }

        return {
          priority,
          confidence: 0.6,
          reasoning: "Based on deadline proximity",
          recommendations: ["Set a specific due date for better prioritization"],
        };
      }
    }),

  // ==================== DEADLINE ESTIMATION ====================

  // Estimate realistic deadline based on task complexity and user's velocity
  estimateDeadline: protectedProcedure
    .input(z.object({
      title: z.string(),
      description: z.string().optional(),
      estimatedTime: z.number().optional(),
      priority: z.enum(["low", "medium", "high", "urgent"]).optional(),
    }))
    .mutation(async ({ input, ctx }) => {
      const tasks = await db.getTasksByUserId(ctx.user.id);
      const completedTasks = tasks.filter(t => t.status === "done");

      // Calculate average completion time
      const avgCompletionDays = completedTasks.length > 0
        ? completedTasks.reduce((sum, task) => {
            const created = new Date(task.createdAt).getTime();
            const updated = new Date(task.updatedAt).getTime();
            return sum + (updated - created) / (1000 * 60 * 60 * 24);
          }, 0) / completedTasks.length
        : 7; // Default to 7 days if no history

      const prompt = `You are an AI deadline estimation assistant. Estimate a realistic deadline for this task.

Task: ${input.title}
Description: ${input.description || "No description"}
Estimated time: ${input.estimatedTime || "unknown"} minutes
Priority: ${input.priority || "not specified"}

User's task completion stats:
- Total tasks: ${tasks.length}
- Completed: ${completedTasks.length}
- Average completion time: ${avgCompletionDays.toFixed(1)} days
- Current active tasks: ${tasks.filter(t => t.status === "in_progress").length}

Respond in JSON format:
{
  "suggestedDeadline": "YYYY-MM-DD",
  "daysFromNow": 7,
  "confidence": 0.0-1.0,
  "reasoning": "Why this deadline is realistic",
  "bufferDays": 2
}

Consider:
1. Task complexity and estimated time
2. User's historical completion rate
3. Current workload
4. Priority level
5. Buffer for unexpected delays

Return only valid JSON, no additional text.`;

      try {
        const response = await openai.chat.completions.create({
          model: "gpt-4.1-mini",
          messages: [{ role: "user", content: prompt }],
          temperature: 0.5,
          max_tokens: 400,
        });

        const content = response.choices[0].message.content || "{}";
        const estimation = JSON.parse(content);

        return estimation;
      } catch (error) {
        console.error("AI deadline estimation error:", error);
        // Fallback to rule-based estimation
        const daysFromNow = Math.ceil(avgCompletionDays * 1.2); // Add 20% buffer
        const suggestedDeadline = new Date();
        suggestedDeadline.setDate(suggestedDeadline.getDate() + daysFromNow);

        return {
          suggestedDeadline: suggestedDeadline.toISOString().split("T")[0],
          daysFromNow,
          confidence: 0.7,
          reasoning: "Based on your average task completion time",
          bufferDays: Math.ceil(daysFromNow * 0.2),
        };
      }
    }),

  // ==================== TASK INSIGHTS ====================

  // Get AI-powered insights and recommendations for task management
  getInsights: protectedProcedure.query(async ({ ctx }) => {
    const tasks = await db.getTasksByUserId(ctx.user.id);
    const now = new Date();

    // Calculate key metrics
    const overdueTasks = tasks.filter(
      t => t.dueDate && new Date(t.dueDate) < now && t.status !== "done"
    );
    const highPriorityTasks = tasks.filter(t => t.priority === "high" || t.priority === "urgent");
    const staleTasks = tasks.filter(
      t => (now.getTime() - new Date(t.updatedAt).getTime()) / (1000 * 60 * 60 * 24) > 7 && t.status !== "done"
    );

    const prompt = `You are an AI productivity coach. Analyze this user's task management and provide insights.

Task Statistics:
- Total tasks: ${tasks.length}
- Completed: ${tasks.filter(t => t.status === "done").length}
- In progress: ${tasks.filter(t => t.status === "in_progress").length}
- Overdue: ${overdueTasks.length}
- High priority: ${highPriorityTasks.length}
- Stale (no update in 7+ days): ${staleTasks.length}

Respond in JSON format:
{
  "insights": [
    {
      "type": "warning|success|info",
      "title": "Insight title",
      "description": "Detailed explanation",
      "actionable": "Specific action to take"
    }
  ],
  "recommendations": [
    "Recommendation 1",
    "Recommendation 2"
  ],
  "focusAreas": ["area1", "area2"]
}

Provide 3-5 actionable insights focusing on:
1. Overdue tasks and time management
2. Priority distribution
3. Task completion patterns
4. Productivity bottlenecks
5. Work-life balance

Return only valid JSON, no additional text.`;

    try {
      const response = await openai.chat.completions.create({
        model: "gpt-4.1-mini",
        messages: [{ role: "user", content: prompt }],
        temperature: 0.6,
        max_tokens: 800,
      });

      const content = response.choices[0].message.content || "{}";
      const insights = JSON.parse(content);

      return insights;
    } catch (error) {
      console.error("AI insights error:", error);
      // Fallback to rule-based insights
      const insights = [];

      if (overdueTasks.length > 0) {
        insights.push({
          type: "warning",
          title: "Overdue Tasks Need Attention",
          description: `You have ${overdueTasks.length} overdue task(s). These should be your top priority.`,
          actionable: "Review overdue tasks and either complete them or adjust their deadlines.",
        });
      }

      if (staleTasks.length > 3) {
        insights.push({
          type: "info",
          title: "Stale Tasks Detected",
          description: `${staleTasks.length} tasks haven't been updated in over a week.`,
          actionable: "Review these tasks and either complete, delegate, or cancel them.",
        });
      }

      if (tasks.filter(t => t.status === "done").length / tasks.length > 0.7) {
        insights.push({
          type: "success",
          title: "Great Completion Rate!",
          description: "You're completing most of your tasks. Keep up the good work!",
          actionable: "Consider taking on more challenging projects.",
        });
      }

      return {
        insights,
        recommendations: [
          "Set specific deadlines for all tasks",
          "Break down large tasks into smaller subtasks",
          "Review and update task priorities weekly",
        ],
        focusAreas: ["time-management", "prioritization"],
      };
    }
  }),

  // ==================== TASK OPTIMIZATION ====================

  // Suggest task optimizations (merge, split, reorder, etc.)
  optimizeTasks: protectedProcedure.mutation(async ({ ctx }) => {
    const tasks = await db.getTasksByUserId(ctx.user.id);

    const prompt = `You are an AI task optimization expert. Analyze these tasks and suggest optimizations.

Tasks:
${tasks.map(t => `- [${t.priority}] ${t.title} (${t.status})`).join("\n")}

Respond in JSON format:
{
  "optimizations": [
    {
      "type": "merge|split|reorder|delegate|cancel",
      "taskIds": [1, 2],
      "suggestion": "What to do",
      "reasoning": "Why this optimization helps",
      "impact": "high|medium|low"
    }
  ]
}

Look for:
1. Similar tasks that can be merged
2. Complex tasks that should be split
3. Tasks that could be delegated
4. Low-value tasks that can be cancelled
5. Better task ordering for efficiency

Return only valid JSON, no additional text.`;

    try {
      const response = await openai.chat.completions.create({
        model: "gpt-4.1-mini",
        messages: [{ role: "user", content: prompt }],
        temperature: 0.6,
        max_tokens: 1000,
      });

      const content = response.choices[0].message.content || "{}";
      const optimizations = JSON.parse(content);

      return optimizations;
    } catch (error) {
      console.error("AI optimization error:", error);
      return {
        optimizations: [],
      };
    }
  }),
});
