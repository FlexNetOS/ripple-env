ALTER TABLE `tasks` ADD `isRecurring` boolean DEFAULT false NOT NULL;--> statement-breakpoint
ALTER TABLE `tasks` ADD `recurrencePattern` varchar(128);--> statement-breakpoint
ALTER TABLE `tasks` ADD `recurrenceEndDate` timestamp;--> statement-breakpoint
ALTER TABLE `tasks` ADD `parentRecurringTaskId` int;