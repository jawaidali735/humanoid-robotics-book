---
id: 2
title: "Auth-Gated Docs Planning"
stage: "plan"
date: "2025-12-23"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "auth-gated-docs"
branch: "1-auth-gated-docs"
user: "user"
command: "/sp.plan"
labels: ["authentication", "authorization", "planning", "architecture", "security"]
links:
  spec: "specs/1-auth-gated-docs/spec.md"
  plan: "specs/1-auth-gated-docs/plan.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/1-auth-gated-docs/plan.md"
  - "specs/1-auth-gated-docs/research.md"
  - "specs/1-auth-gated-docs/data-model.md"
  - "specs/1-auth-gated-docs/contracts/auth-api.yaml"
  - "specs/1-auth-gated-docs/quickstart.md"
  - ".claude/agent-context.md"
tests: []
---

# PHR: Auth-Gated Docs Planning

## Prompt Text

```
/sp.plan

Plan the implementation of the **Auth-Gated Docs & Chatbot system** based strictly on the provided spec.
Do not introduce new features, tools, or assumptions beyond the given content.

---

## 1. Architecture Sketch

### High-Level Architecture
- Existing `backend/` remains unchanged
- New isolated folder:

- `backend/auth/` handles:
- Authentication logic
- Session validation
- Integration points for docs access & chatbot access
- Docusaurus frontend and chatbot frontend **only consume auth state**
- Neon PostgreSQL stores users and sessions
- Better Auth manages login, password hashing, and sessions

---

## 2. Section Structure (By Responsibility)

### Backend (`backend/auth/`)
- Auth initialization (Better Auth)
- Login endpoint
- Logout endpoint
- Session validation middleware
- Auth check endpoint for:
- Docs access
- Chatbot access

### Database (Neon PostgreSQL)
- `users` table
- `sessions` table

### Frontend
- Auth state handling:
- `loading`
- `unauthenticated`
- `authenticated`
- Docs locking UI (blur/overlay + CTA)
- Chatbot gating UI (disabled input until login)

---

## 3. Research Approach (Research-Concurrent)

- While implementing:
- Refer to **Better Auth documentation** for:
  - Email/password auth
  - Session handling
  - Cookie-based auth
- Refer to **Neon PostgreSQL documentation** for:
  - Database connection
  - Table setup
- Research happens **alongside implementation**, not upfront
- Only features explicitly required by the spec are researched

---

## 4. Decisions Needing Documentation

### Decision 1: Auth Isolation
- **Chosen:** Isolated `backend/auth/` folder
- **Alternative:** Modifying existing backend
- **Tradeoff:**
- Isolation avoids breaking existing code
- Slight integration effort needed via middleware

### Decision 2: Session-Based Auth
- **Chosen:** Session-based auth with HttpOnly cookies
- **Alternative:** Pure token-based client auth
- **Tradeoff:**
- Sessions are more secure for gated content
- Requires backend validation on every request

### Decision 3: Content Gating Method
- **Chosen:** UI-level lock + backend validation
- **Alternative:** Frontend-only lock
- **Tradeoff:**
- Backend validation prevents bypassing locks
- Slight increase in request checks

---

## 5. Testing Strategy (Based on Acceptance Criteria)

### Auth Tests
- Attempt login with valid credentials → success
- Attempt login with invalid credentials → failure

### Docs Access Tests
- Open main chapter without login → content hidden
- Login → content immediately visible
- Refresh page → content remains visible

### Chatbot Tests
- Open chatbot without login → input disabled
- Login → input enabled
- Send chat request without session → rejected by backend

### Backend Safety Tests
- Existing backend functionality remains unaffected
- All auth logic exists only inside `backend/auth/`

---

## 6. Quality Validation

- Verify no existing backend files were modified
- Confirm all auth logic lives inside `backend/auth/`
- Validate Neon DB stores users and sessions corr
```

## Response Text

Completed comprehensive planning for the Auth-Gated Docs & Chatbot feature:

1. Created detailed implementation plan with technical context, constitution check, and gates
2. Researched and resolved all unknowns about Better Auth integration, Neon PostgreSQL setup, Docusaurus authentication patterns, and chatbot API integration
3. Designed data models for User and Session entities with proper relationships and validation rules
4. Created API contract specification for authentication endpoints
5. Developed quickstart guide for developers
6. Created agent context file with key information about the feature

All planning artifacts are stored in the specs/1-auth-gated-docs/ directory and follow the specified architecture of isolated auth module that doesn't modify existing backend code.

## Outcome

Planning phase successfully completed with all required artifacts:
- Implementation plan (plan.md)
- Research findings (research.md)
- Data models (data-model.md)
- API contracts (contracts/auth-api.yaml)
- Quickstart guide (quickstart.md)
- Agent context (agent-context.md)

## Evaluation

The plan aligns with all specified requirements and constraints. The architecture maintains isolation of auth logic while providing necessary integration points with existing backend functionality. All research tasks were completed successfully, resolving initial unknowns about technology integration.