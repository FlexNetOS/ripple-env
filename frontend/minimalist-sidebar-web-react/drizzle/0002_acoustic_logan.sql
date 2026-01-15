CREATE TABLE `favorites` (
	`id` int AUTO_INCREMENT NOT NULL,
	`userId` int NOT NULL,
	`itemId` varchar(128) NOT NULL,
	`itemType` varchar(64) NOT NULL,
	`title` varchar(255) NOT NULL,
	`icon` varchar(64),
	`route` varchar(255),
	`panelId` varchar(128),
	`sortOrder` int NOT NULL DEFAULT 0,
	`createdAt` timestamp NOT NULL DEFAULT (now()),
	CONSTRAINT `favorites_id` PRIMARY KEY(`id`)
);
