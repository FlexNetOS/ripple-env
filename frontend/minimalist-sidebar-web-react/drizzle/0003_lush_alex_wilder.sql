CREATE TABLE `task_activities` (
	`id` int AUTO_INCREMENT NOT NULL,
	`taskId` int NOT NULL,
	`type` enum('created','updated','commented','status_changed','assigned') NOT NULL,
	`user` varchar(255) NOT NULL,
	`description` text NOT NULL,
	`createdAt` timestamp NOT NULL DEFAULT (now()),
	CONSTRAINT `task_activities_id` PRIMARY KEY(`id`)
);
--> statement-breakpoint
CREATE TABLE `task_comments` (
	`id` int AUTO_INCREMENT NOT NULL,
	`taskId` int NOT NULL,
	`author` varchar(255) NOT NULL,
	`content` text NOT NULL,
	`avatar` text,
	`createdAt` timestamp NOT NULL DEFAULT (now()),
	CONSTRAINT `task_comments_id` PRIMARY KEY(`id`)
);
--> statement-breakpoint
CREATE TABLE `task_dependencies` (
	`id` int AUTO_INCREMENT NOT NULL,
	`taskId` int NOT NULL,
	`dependsOnTaskId` int NOT NULL,
	`createdAt` timestamp NOT NULL DEFAULT (now()),
	CONSTRAINT `task_dependencies_id` PRIMARY KEY(`id`)
);
--> statement-breakpoint
CREATE TABLE `task_time_entries` (
	`id` int AUTO_INCREMENT NOT NULL,
	`taskId` int NOT NULL,
	`duration` int NOT NULL,
	`description` text NOT NULL,
	`createdAt` timestamp NOT NULL DEFAULT (now()),
	CONSTRAINT `task_time_entries_id` PRIMARY KEY(`id`)
);
--> statement-breakpoint
CREATE TABLE `tasks` (
	`id` int AUTO_INCREMENT NOT NULL,
	`userId` int NOT NULL,
	`title` varchar(255) NOT NULL,
	`description` text,
	`status` enum('todo','in_progress','review','done','cancelled') NOT NULL DEFAULT 'todo',
	`priority` enum('low','medium','high','urgent') NOT NULL DEFAULT 'medium',
	`assignee` varchar(255),
	`dueDate` timestamp,
	`tags` json DEFAULT ('[]'),
	`estimatedTime` int,
	`progress` int NOT NULL DEFAULT 0,
	`starred` boolean NOT NULL DEFAULT false,
	`parentTaskId` int,
	`sharedTaskId` varchar(128),
	`projectId` varchar(128),
	`hasInProgressAttempt` boolean NOT NULL DEFAULT false,
	`lastAttemptFailed` boolean NOT NULL DEFAULT false,
	`executor` varchar(128),
	`createdAt` timestamp NOT NULL DEFAULT (now()),
	`updatedAt` timestamp NOT NULL DEFAULT (now()) ON UPDATE CURRENT_TIMESTAMP,
	CONSTRAINT `tasks_id` PRIMARY KEY(`id`)
);
