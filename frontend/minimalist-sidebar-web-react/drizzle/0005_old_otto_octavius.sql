ALTER TABLE `tasks` MODIFY COLUMN `status` enum('backlog','todo','in_progress','review','done','cancelled') NOT NULL DEFAULT 'todo';--> statement-breakpoint
ALTER TABLE `tasks` ADD `taskId` varchar(64) NOT NULL;--> statement-breakpoint
ALTER TABLE `tasks` ADD CONSTRAINT `tasks_taskId_unique` UNIQUE(`taskId`);