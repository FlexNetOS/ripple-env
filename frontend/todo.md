# Project TODO

## Core Infrastructure
- [x] Database schema with users, files, notifications, subscriptions, chat tables
- [x] Push database migrations with Drizzle ORM

## Authentication & Authorization
- [x] Manus OAuth integration (pre-configured)
- [x] Session management with HTTP-only cookies (pre-configured)
- [x] Role-based access control (admin/user roles)
- [x] Protected routes and procedures

## File Storage
- [x] S3 file upload integration
- [x] File metadata storage in database
- [x] File listing and management API
- [x] Secure file access with user ownership

## AI Chatbot
- [x] LLM integration using built-in invokeLLM
- [x] Chat history storage in database
- [x] Chat conversation management
- [x] Chat UI with markdown rendering (Streamdown)

## Notification System
- [x] In-app notifications table and API
- [x] Owner notification integration
- [x] Notification preferences
- [x] Notification display and management

## Payment Integration
- [x] Stripe integration setup
- [x] Subscription plans and pricing
- [x] Payment webhook handling
- [x] Subscription status management
- [x] Customer portal session

## Frontend UI
- [x] Elegant purple/violet theme and color palette
- [x] Landing page with hero section
- [x] Dashboard layout with sidebar navigation
- [x] Dashboard overview with stats
- [x] AI Chat interface with conversation list
- [x] File manager page with upload/delete
- [x] Notifications page with read/unread
- [x] Billing page with subscription management
- [x] Pricing page (public)
- [x] Settings page with profile and preferences

## Testing
- [x] Auth logout test (pre-configured)
- [x] User profile API tests
- [x] Notification API tests
- [x] Subscription API tests

## Polish
- [x] Responsive design for all pages
- [x] Loading states and skeletons
- [x] Error handling and toast notifications
- [x] Empty states for all lists


## Integration Tasks (from app.tar)
- [x] Extract app.tar to project root
- [x] Copy assets to client/public/images
- [x] Copy original sidebar-panels.ts data (all 16 panels)
- [x] Update IconNavigation to match original structure
- [x] Update DetailSidebar to match original structure
- [x] Update SidebarContext with proper workspace types
- [x] Preserve exact color scheme from theme.config.js
- [x] Set up dynamic-components registry directory
- [x] Update TwoLevelSidebar to wrap dashboard routes
- [x] Update App.tsx with sidebar wrapper


## Bug Fixes
- [x] Fix detail sidebar (second panel) not flexing out correctly


## Sidebar Enhancements
- [x] NotificationsPanel modal overlay
- [x] FavoritesPanel modal overlay
- [x] Route-based panel auto-selection
- [x] Ripple logo branding (replace Nexus)
- [x] Collapse/expand toggle on icon rail
- [x] ⌘K command palette modal


## Navigation & Branding Fixes
- [x] Fix navigation bar overlap with two-level sidebar (integrate nav into sidebar layout)
- [x] Update landing page header with ripple logo (replace "Nexus" text)
- [x] Add keyboard navigation to sidebar panels (arrow keys + Enter)
- [x] Connect NotificationsPanel to real tRPC notification endpoints


## New Enhancements (Round 3)
- [x] Real-time notification updates with polling
- [x] FavoritesPanel with database persistence
- [x] User avatar upload in Settings page
- [x] Themed scrollbar colors matching ripple brand


## New Enhancements (Round 4)
- [ ] Drag-and-drop file upload in Files page
- [ ] Workspace switching functionality in sidebar
- [ ] Real-time toast notifications for events
