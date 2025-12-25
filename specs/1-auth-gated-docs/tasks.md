# Implementation Tasks: Auth-Gated Docs & Chatbot

**Feature**: Auth-Gated Docs & Chatbot
**Branch**: 1-auth-gated-docs
**Created**: 2025-12-23
**Status**: Draft
**Spec**: specs/1-auth-gated-docs/spec.md
**Plan**: specs/1-auth-gated-docs/plan.md

## Phase 1: Setup

### Goal
Initialize project structure and configure dependencies for the authentication system.

### Tasks
- [x] T001 Create backend/auth directory structure
- [x] T002 Install Better Auth and related dependencies
- [x] T003 Install Neon PostgreSQL client dependencies
- [x] T004 Create initial configuration files for auth system
- [x] T005 Set up environment variables for auth system in existing env file

## Phase 2: Foundational

### Goal
Implement core authentication infrastructure that all user stories depend on.

### Tasks
- [x] T006 [P] Create User entity model in backend/auth/models/user.js but check how beter auth frameworkd workd accordingly setup everything.
- [x] T007 [P] Create Session entity model in backend/auth/models/session.js
- [x] T008 [P] Set up Neon PostgreSQL database connection in backend/auth/config/database.js
- [x] T009 [P] Create database migration scripts for users and sessions tables
- [x] T010 [P] Implement Better Auth configuration in backend/auth/config/auth.js
- [x] T011 [P] Create authentication middleware in backend/auth/middleware/auth-guard.js
- [x] T012 Create API response utility functions in backend/auth/utils/response.js
- [x] T013 Create input validation utility functions in backend/auth/utils/validators.js

## Phase 3: [US1] Secure Access to Book Content

### Goal
Implement content access control that requires users to log in to access main book chapters and advanced sections.

### Independent Test
Can be fully tested by attempting to access book chapters without being logged in and verifying that content is blocked with a login overlay, while still showing chapter titles.

### Tasks
- [x] T014 [P] [US1] Create content access validation endpoint in backend/auth/routes/content-access.js
- [x] T015 [P] [US1] Implement content access middleware in backend/auth/middleware/content-guard.js
- [x] T016 [US1] Create frontend authentication context in src/contexts/AuthContext.js
- [x] T017 [US1] Implement protected content component in src/components/Content/ProtectedContent.js
- [x] T018 [US1] Create content gating overlay UI in src/components/Content/ContentGate.js
- [x] T019 [US1] Add "Login to continue" CTA to protected content
- [x] T020 [US1] Implement content access check in Docusaurus pages
- [x] T021 [US1] Test content access control with unauthenticated users

## Phase 4: [US2] Secure Chatbot Access

### Goal
Implement chatbot access control that requires users to log in to use the AI-powered Q&A functionality.

### Independent Test
Can be fully tested by attempting to use the chatbot without being logged in and verifying that input is disabled with a "Please login" message.

### Tasks
- [x] T022 [P] [US2] Create chat access validation endpoint in backend/auth/routes/chat-access.js
- [x] T023 [P] [US2] Implement chat access middleware in backend/auth/middleware/chat-guard.js
- [x] T024 [P] [US2] Modify existing chatbot API to validate authentication in backend/api/chat.py
- [x] T025 [US2] Update chatbot UI to check authentication state in src/components/ChatInterface/ChatInterface.js
- [x] T026 [US2] Implement disabled input state for unauthenticated users
- [x] T027 [US2] Add "Please login" message to chatbot interface
- [x] T028 [US2] Attach user_id to authenticated chat requests
- [x] T029 [US2] Test chatbot access control with unauthenticated users

## Phase 5: [US3] User Registration and Authentication

### Goal
Implement user registration with email and password so that new users can access the protected content and services.

### Independent Test
Can be fully tested by creating a new account and verifying that the user can log in and access protected content.

### Tasks
- [x] T030 [P] [US3] Create registration endpoint in backend/auth/routes/register.js
- [x] T031 [P] [US3] Create login endpoint in backend/auth/routes/login.js
- [x] T032 [P] [US3] Create logout endpoint in backend/auth/routes/logout.js
- [x] T033 [P] [US3] Create session check endpoint in backend/auth/routes/session.js
- [x] T034 [US3] Implement user registration service in backend/auth/services/user-service.js
- [x] T035 [US3] Implement user authentication service in backend/auth/services/auth-service.js
- [x] T036 [US3] Create authentication form components in src/components/Auth/
- [x] T037 [US3] Implement session persistence across page refreshes
- [x] T038 [US3] Test user registration and login flows
- [x] T039 [US3] Test session persistence across page refreshes

## Phase 6: [US4] Public Content Access

### Goal
Ensure that visitors can access public content like the landing page and introduction without authentication.

### Independent Test
Can be fully tested by accessing public pages without authentication and verifying they're fully visible.

### Tasks
- [x] T040 [P] [US4] Identify public content routes that should remain accessible
- [x] T041 [P] [US4] Create public content middleware in backend/auth/middleware/public-guard.js
- [x] T042 [US4] Ensure landing page content remains visible to unauthenticated users
- [x] T043 [US4] Ensure intro section remains visible to unauthenticated users
- [x] T044 [US4] Test public content accessibility for unauthenticated users
- [x] T045 [US4] Verify advanced sections remain blocked for unauthenticated users

## Phase 7: Polish & Cross-Cutting Concerns

### Goal
Final integration, testing, and polish to ensure all components work together seamlessly.

### Tasks
- [ ] T046 Integrate auth module with existing backend without breaking changes
- [ ] T047 Add comprehensive error handling and logging
- [ ] T048 Implement rate limiting for auth endpoints
- [ ] T049 Add security headers and CSRF protection
- [ ] T050 Perform security validation of auth flows
- [ ] T051 Conduct performance testing of auth validation
- [ ] T052 Write comprehensive integration tests
- [ ] T053 Update documentation for auth system
- [ ] T054 Perform end-to-end testing of all user flows


## Dependencies

### User Story Completion Order
1. US3 (User Registration and Authentication) - foundational for all other stories
2. US1 (Secure Access to Book Content) - requires auth system
3. US2 (Secure Chatbot Access) - requires auth system
4. US4 (Public Content Access) - verification of public content

### Critical Path
- T001 → T002 → T006 → T008 → T010 → T030 → T031 → T014 → T022

## Parallel Execution Examples

### Per Story
- **US1**: T014, T015 can run in parallel with T016, T017, T018
- **US2**: T022, T023, T024 can run in parallel with T025, T026, T027
- **US3**: T030, T031, T032, T033 can run in parallel with T034, T035
- **US4**: T040, T041 can run in parallel with T042, T043

## Implementation Strategy

### MVP First Approach
1. Start with US3 (User Registration and Authentication) - minimum viable auth system
2. Add US1 (Content Access Control) - basic protection
3. Add US2 (Chatbot Access Control) - protected functionality
4. Add US4 (Public Content Access) - ensure public content still works

### Incremental Delivery
- **MVP**: User registration/login + basic content protection
- **Phase 2**: Chatbot access control
- **Phase 3**: Public content verification and polish