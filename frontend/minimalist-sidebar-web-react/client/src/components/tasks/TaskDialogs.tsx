import { useState } from "react";
import { motion } from "framer-motion";
import {
  Clock,
  User,
  Tag,
  Calendar,
  AlertCircle,
  CheckCircle2,
  Timer,
  Zap,
  Star,
  MessageSquare,
  Activity,
  Plus,
  X,
} from "lucide-react";
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogDescription, DialogFooter } from "@/components/ui/dialog";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Textarea } from "@/components/ui/textarea";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Badge } from "@/components/ui/badge";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Calendar as CalendarComponent } from "@/components/ui/calendar";
import { Popover, PopoverContent, PopoverTrigger } from "@/components/ui/popover";
import { Progress } from "@/components/ui/progress";
import { Avatar, AvatarFallback, AvatarImage } from "@/components/ui/avatar";
import { Separator } from "@/components/ui/separator";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";

type TaskStatus = "todo" | "in_progress" | "review" | "done";
type TaskPriority = "low" | "medium" | "high" | "urgent";

interface Comment {
  id: string;
  author: string;
  content: string;
  timestamp: Date;
  avatar?: string;
}

interface Activity {
  id: string;
  type: "created" | "updated" | "commented" | "status_changed" | "assigned";
  user: string;
  description: string;
  timestamp: Date;
}

interface TimeEntry {
  id: string;
  duration: number;
  description: string;
  timestamp: Date;
}

interface Task {
  id: string;
  title: string;
  description: string;
  status: TaskStatus;
  priority: TaskPriority;
  assignee?: string;
  dueDate?: Date;
  tags: string[];
  comments: Comment[];
  activities: Activity[];
  timeEntries: TimeEntry[];
  estimatedTime?: number;
  progress: number;
  dependencies: string[];
  starred: boolean;
  createdAt: Date;
  updatedAt: Date;
}

const statusConfig: Record<TaskStatus, { label: string; color: string; bgColor: string; icon: React.ReactNode }> = {
  todo: { label: "To Do", color: "text-gray-400", bgColor: "bg-gray-500/10", icon: <AlertCircle className="w-4 h-4" /> },
  in_progress: { label: "In Progress", color: "text-[#00D4FF]", bgColor: "bg-[#00D4FF]/10", icon: <Timer className="w-4 h-4" /> },
  review: { label: "Review", color: "text-[#9B7BFF]", bgColor: "bg-[#9B7BFF]/10", icon: <Zap className="w-4 h-4" /> },
  done: { label: "Done", color: "text-[#00E676]", bgColor: "bg-[#00E676]/10", icon: <CheckCircle2 className="w-4 h-4" /> },
};

const priorityConfig: Record<TaskPriority, { label: string; color: string; bgColor: string }> = {
  low: { label: "Low", color: "text-gray-400", bgColor: "bg-gray-500/20" },
  medium: { label: "Medium", color: "text-[#00D4FF]", bgColor: "bg-[#00D4FF]/20" },
  high: { label: "High", color: "text-orange-400", bgColor: "bg-orange-500/20" },
  urgent: { label: "Urgent", color: "text-red-400", bgColor: "bg-red-500/20" },
};

// Task Detail Dialog
export function TaskDetailDialog({
  task,
  open,
  onOpenChange,
  onEdit,
}: {
  task: Task | null;
  open: boolean;
  onOpenChange: (open: boolean) => void;
  onEdit: (task: Task) => void;
}) {
  const [newComment, setNewComment] = useState("");

  if (!task) return null;

  const totalTimeSpent = task.timeEntries.reduce((acc, entry) => acc + entry.duration, 0);
  const timeProgress = task.estimatedTime ? Math.min((totalTimeSpent / task.estimatedTime) * 100, 100) : 0;

  const handleAddComment = () => {
    if (!newComment.trim()) return;
    // Add comment logic here
    setNewComment("");
  };

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent className="max-w-4xl max-h-[90vh] p-0">
        <div className="flex flex-col h-full">
          {/* Header */}
          <DialogHeader className="px-6 py-4 border-b border-[#2A2A2A]">
            <div className="flex items-start justify-between gap-4">
              <div className="flex-1">
                <div className="flex items-center gap-2 mb-2">
                  {task.starred && <Star className="w-5 h-5 text-yellow-400 fill-yellow-400" />}
                  <DialogTitle className="text-2xl">{task.title}</DialogTitle>
                </div>
                <div className="flex items-center gap-2 flex-wrap">
                  <Badge className={`${statusConfig[task.status].bgColor} ${statusConfig[task.status].color}`}>
                    {statusConfig[task.status].label}
                  </Badge>
                  <Badge className={`${priorityConfig[task.priority].bgColor} ${priorityConfig[task.priority].color}`}>
                    {priorityConfig[task.priority].label}
                  </Badge>
                  {task.tags.map((tag) => (
                    <Badge key={tag} variant="secondary">
                      {tag}
                    </Badge>
                  ))}
                </div>
              </div>
              <Button variant="outline" size="sm" onClick={() => onEdit(task)}>
                Edit Task
              </Button>
            </div>
          </DialogHeader>

          {/* Content */}
          <ScrollArea className="flex-1 px-6 py-4">
            <Tabs defaultValue="overview" className="w-full">
              <TabsList className="mb-4">
                <TabsTrigger value="overview">Overview</TabsTrigger>
                <TabsTrigger value="comments">
                  Comments ({task.comments.length})
                </TabsTrigger>
                <TabsTrigger value="activity">
                  Activity ({task.activities.length})
                </TabsTrigger>
                <TabsTrigger value="time">Time Tracking</TabsTrigger>
              </TabsList>

              <TabsContent value="overview" className="space-y-6">
                {/* Description */}
                <div>
                  <h3 className="text-sm font-semibold text-gray-400 mb-2">Description</h3>
                  <p className="text-white">{task.description}</p>
                </div>

                {/* Details Grid */}
                <div className="grid grid-cols-2 gap-4">
                  <div>
                    <h3 className="text-sm font-semibold text-gray-400 mb-2">Assignee</h3>
                    <div className="flex items-center gap-2">
                      <Avatar className="w-8 h-8">
                        <AvatarFallback>{task.assignee?.[0] || "?"}</AvatarFallback>
                      </Avatar>
                      <span className="text-white">{task.assignee || "Unassigned"}</span>
                    </div>
                  </div>

                  <div>
                    <h3 className="text-sm font-semibold text-gray-400 mb-2">Due Date</h3>
                    <div className="flex items-center gap-2 text-white">
                      <Calendar className="w-4 h-4" />
                      <span>{task.dueDate?.toLocaleDateString() || "No due date"}</span>
                    </div>
                  </div>

                  <div>
                    <h3 className="text-sm font-semibold text-gray-400 mb-2">Created</h3>
                    <span className="text-white">{task.createdAt.toLocaleString()}</span>
                  </div>

                  <div>
                    <h3 className="text-sm font-semibold text-gray-400 mb-2">Last Updated</h3>
                    <span className="text-white">{task.updatedAt.toLocaleString()}</span>
                  </div>
                </div>

                {/* Progress */}
                <div>
                  <div className="flex items-center justify-between mb-2">
                    <h3 className="text-sm font-semibold text-gray-400">Progress</h3>
                    <span className="text-white font-medium">{task.progress}%</span>
                  </div>
                  <Progress value={task.progress} className="h-2" />
                </div>

                {/* Time Tracking Summary */}
                {task.estimatedTime && (
                  <div>
                    <div className="flex items-center justify-between mb-2">
                      <h3 className="text-sm font-semibold text-gray-400">Time Tracking</h3>
                      <span className="text-white">
                        {Math.floor(totalTimeSpent / 60)}h {totalTimeSpent % 60}m / {Math.floor(task.estimatedTime / 60)}h
                      </span>
                    </div>
                    <Progress value={timeProgress} className="h-2" />
                  </div>
                )}

                {/* Dependencies */}
                {task.dependencies.length > 0 && (
                  <div>
                    <h3 className="text-sm font-semibold text-gray-400 mb-2">Dependencies</h3>
                    <div className="space-y-2">
                      {task.dependencies.map((depId) => (
                        <div key={depId} className="flex items-center gap-2 text-sm text-gray-300">
                          <AlertCircle className="w-4 h-4" />
                          <span>Task #{depId}</span>
                        </div>
                      ))}
                    </div>
                  </div>
                )}
              </TabsContent>

              <TabsContent value="comments" className="space-y-4">
                {/* Add Comment */}
                <div className="flex gap-2">
                  <Avatar className="w-8 h-8">
                    <AvatarFallback>DU</AvatarFallback>
                  </Avatar>
                  <div className="flex-1 space-y-2">
                    <Textarea
                      placeholder="Add a comment..."
                      value={newComment}
                      onChange={(e) => setNewComment(e.target.value)}
                      className="min-h-[80px]"
                    />
                    <Button size="sm" onClick={handleAddComment}>
                      <Plus className="w-4 h-4 mr-2" />
                      Add Comment
                    </Button>
                  </div>
                </div>

                <Separator />

                {/* Comments List */}
                <div className="space-y-4">
                  {task.comments.map((comment) => (
                    <div key={comment.id} className="flex gap-3">
                      <Avatar className="w-8 h-8">
                        <AvatarFallback>{comment.author[0]}</AvatarFallback>
                      </Avatar>
                      <div className="flex-1">
                        <div className="flex items-center gap-2 mb-1">
                          <span className="font-medium text-white">{comment.author}</span>
                          <span className="text-xs text-gray-500">
                            {comment.timestamp.toLocaleString()}
                          </span>
                        </div>
                        <p className="text-gray-300">{comment.content}</p>
                      </div>
                    </div>
                  ))}
                </div>
              </TabsContent>

              <TabsContent value="activity" className="space-y-3">
                {task.activities.map((activity) => (
                  <div key={activity.id} className="flex gap-3">
                    <div className="flex-shrink-0 w-8 h-8 rounded-full bg-[#2A2A2A] flex items-center justify-center">
                      <Activity className="w-4 h-4 text-[#00D4FF]" />
                    </div>
                    <div className="flex-1">
                      <p className="text-white">
                        <span className="font-medium">{activity.user}</span>{" "}
                        <span className="text-gray-400">{activity.description}</span>
                      </p>
                      <span className="text-xs text-gray-500">
                        {activity.timestamp.toLocaleString()}
                      </span>
                    </div>
                  </div>
                ))}
              </TabsContent>

              <TabsContent value="time" className="space-y-4">
                {/* Time Entries */}
                <div className="space-y-3">
                  {task.timeEntries.map((entry) => (
                    <div
                      key={entry.id}
                      className="flex items-center justify-between p-3 bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg"
                    >
                      <div className="flex-1">
                        <p className="text-white font-medium">{entry.description}</p>
                        <span className="text-xs text-gray-500">
                          {entry.timestamp.toLocaleString()}
                        </span>
                      </div>
                      <div className="flex items-center gap-2 text-[#00D4FF]">
                        <Clock className="w-4 h-4" />
                        <span className="font-medium">
                          {Math.floor(entry.duration / 60)}h {entry.duration % 60}m
                        </span>
                      </div>
                    </div>
                  ))}
                </div>

                {/* Add Time Entry */}
                <Button variant="outline" className="w-full">
                  <Plus className="w-4 h-4 mr-2" />
                  Add Time Entry
                </Button>
              </TabsContent>
            </Tabs>
          </ScrollArea>
        </div>
      </DialogContent>
    </Dialog>
  );
}

// Task Create/Edit Dialog
export function TaskFormDialog({
  task,
  open,
  onOpenChange,
  onSave,
}: {
  task: Task | null;
  open: boolean;
  onOpenChange: (open: boolean) => void;
  onSave: (task: Task) => void;
}) {
  const [formData, setFormData] = useState<Partial<Task>>(task || {});
  const [newTag, setNewTag] = useState("");

  const handleSave = () => {
    if (!formData.title?.trim()) return;
    onSave(formData as Task);
  };

  const handleAddTag = () => {
    if (!newTag.trim() || formData.tags?.includes(newTag)) return;
    setFormData({
      ...formData,
      tags: [...(formData.tags || []), newTag],
    });
    setNewTag("");
  };

  const handleRemoveTag = (tag: string) => {
    setFormData({
      ...formData,
      tags: formData.tags?.filter((t) => t !== tag) || [],
    });
  };

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent className="max-w-2xl">
        <DialogHeader>
          <DialogTitle>{task?.id ? "Edit Task" : "Create New Task"}</DialogTitle>
          <DialogDescription>
            {task?.id ? "Update task details" : "Add a new task to your board"}
          </DialogDescription>
        </DialogHeader>

        <div className="space-y-4 py-4">
          {/* Title */}
          <div className="space-y-2">
            <Label htmlFor="title">Title *</Label>
            <Input
              id="title"
              placeholder="Enter task title..."
              value={formData.title || ""}
              onChange={(e) => setFormData({ ...formData, title: e.target.value })}
            />
          </div>

          {/* Description */}
          <div className="space-y-2">
            <Label htmlFor="description">Description</Label>
            <Textarea
              id="description"
              placeholder="Enter task description..."
              value={formData.description || ""}
              onChange={(e) => setFormData({ ...formData, description: e.target.value })}
              className="min-h-[100px]"
            />
          </div>

          {/* Status and Priority */}
          <div className="grid grid-cols-2 gap-4">
            <div className="space-y-2">
              <Label>Status</Label>
              <Select
                value={formData.status || "todo"}
                onValueChange={(value) => setFormData({ ...formData, status: value as TaskStatus })}
              >
                <SelectTrigger>
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  {Object.entries(statusConfig).map(([key, config]) => (
                    <SelectItem key={key} value={key}>
                      {config.label}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
            </div>

            <div className="space-y-2">
              <Label>Priority</Label>
              <Select
                value={formData.priority || "medium"}
                onValueChange={(value) => setFormData({ ...formData, priority: value as TaskPriority })}
              >
                <SelectTrigger>
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  {Object.entries(priorityConfig).map(([key, config]) => (
                    <SelectItem key={key} value={key}>
                      {config.label}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
            </div>
          </div>

          {/* Assignee and Due Date */}
          <div className="grid grid-cols-2 gap-4">
            <div className="space-y-2">
              <Label htmlFor="assignee">Assignee</Label>
              <Input
                id="assignee"
                placeholder="Assign to..."
                value={formData.assignee || ""}
                onChange={(e) => setFormData({ ...formData, assignee: e.target.value })}
              />
            </div>

            <div className="space-y-2">
              <Label>Due Date</Label>
              <Popover>
                <PopoverTrigger asChild>
                  <Button variant="outline" className="w-full justify-start text-left font-normal">
                    <Calendar className="mr-2 h-4 w-4" />
                    {formData.dueDate ? formData.dueDate.toLocaleDateString() : "Pick a date"}
                  </Button>
                </PopoverTrigger>
                <PopoverContent className="w-auto p-0" align="start">
                  <CalendarComponent
                    mode="single"
                    selected={formData.dueDate}
                    onSelect={(date) => setFormData({ ...formData, dueDate: date })}
                  />
                </PopoverContent>
              </Popover>
            </div>
          </div>

          {/* Progress */}
          <div className="space-y-2">
            <div className="flex items-center justify-between">
              <Label>Progress</Label>
              <span className="text-sm text-gray-400">{formData.progress || 0}%</span>
            </div>
            <Slider
              value={[formData.progress || 0]}
              onValueChange={([value]) => setFormData({ ...formData, progress: value })}
              max={100}
              step={5}
            />
          </div>

          {/* Estimated Time */}
          <div className="space-y-2">
            <Label htmlFor="estimatedTime">Estimated Time (hours)</Label>
            <Input
              id="estimatedTime"
              type="number"
              placeholder="0"
              value={formData.estimatedTime ? formData.estimatedTime / 60 : ""}
              onChange={(e) =>
                setFormData({ ...formData, estimatedTime: parseInt(e.target.value) * 60 || 0 })
              }
            />
          </div>

          {/* Tags */}
          <div className="space-y-2">
            <Label>Tags</Label>
            <div className="flex gap-2">
              <Input
                placeholder="Add tag..."
                value={newTag}
                onChange={(e) => setNewTag(e.target.value)}
                onKeyPress={(e) => e.key === "Enter" && handleAddTag()}
              />
              <Button type="button" onClick={handleAddTag}>
                <Plus className="w-4 h-4" />
              </Button>
            </div>
            {formData.tags && formData.tags.length > 0 && (
              <div className="flex flex-wrap gap-2 mt-2">
                {formData.tags.map((tag) => (
                  <Badge key={tag} variant="secondary" className="gap-1">
                    {tag}
                    <X
                      className="w-3 h-3 cursor-pointer"
                      onClick={() => handleRemoveTag(tag)}
                    />
                  </Badge>
                ))}
              </div>
            )}
          </div>
        </div>

        <DialogFooter>
          <Button variant="outline" onClick={() => onOpenChange(false)}>
            Cancel
          </Button>
          <Button onClick={handleSave} disabled={!formData.title?.trim()}>
            {task?.id ? "Update Task" : "Create Task"}
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
}

export default TaskDialogs;
