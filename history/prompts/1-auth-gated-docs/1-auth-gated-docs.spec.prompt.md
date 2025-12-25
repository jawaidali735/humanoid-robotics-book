---
id: 1
title: "Auth-Gated Docs Specification"
stage: "spec"
date: "2025-12-23"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "auth-gated-docs"
branch: "1-auth-gated-docs"
user: "user"
command: "/sp.specify"
labels: ["authentication", "authorization", "docs", "chatbot", "security"]
links:
  spec: "specs/1-auth-gated-docs/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/1-auth-gated-docs/spec.md"
  - "specs/1-auth-gated-docs/checklists/requirements.md"
tests: []
---

# PHR: Auth-Gated Docs Specification

## Prompt Text

```
## 🔐 Auth-Gated Docs & Chatbot — System Prompt (Spec)

Build a secure authentication system for a Docusaurus-based book website with an attached chatbot.

---

### 📁 Backend Constraint (IMPORTANT)
- A **backend folder already exists**
- **Do NOT modify or break existing backend code**
- Create a **new isolated folder named `auth/`**
- Implement the **entire authentication system only inside `backend/auth/`**
- Existing RAG, embeddings, retrieval, and chatbot logic must remain untouched
- Auth must integrate **without refactoring existing modules**

---

### 🎯 Goal
- Lock main book chapters and chatbot access
- Require user login to unlock content
- Use **Better Auth** for authentication
- Store user and session data in **Neon PostgreSQL**
- Show content strictly based on user auth state

---

### 🔑 Authentication
- Provider: Better Auth
- Login: Email + Password
- Session-based auth (JWT / HttpOnly cookies)
- Persist session on page refresh

---

## 📌 References (If system does not know these tools)
- **Better Auth Docs:** https://www.better-auth.com/
- **Neon PostgreSQL Docs:** https://neon.com/

Use the above links to understand Better Auth features and Neon serverless Postgres setup.

---

### 🗄️ Database (Neon DB)
**users**
- id (uuid, PK)
- email (unique)
- name
- password_hash
- role (guest/user/admin)
- created_at

**sessions**
- id (uuid)
- user_id (FK)
- expires_at

---

### 🔒 Access Rules
- Public: Landing page, intro
- Locked: Main chapters, advanced sections, downloads
- Chatbot:
  - Logged out → input disabled + "Please login"
  - Logged in → full access

---

### 📚 Docs Locking (Docusaurus)
- Show chapter titles
- Hide content behind blur/overlay
- CTA: **"Login to continue"**

---

### 🤖 Chatbot Gating
- Block chat requests without a valid session
- Attach `user_id` to every chat request
- Reject unauthenticated requests at API level

---

### 🔁 Frontend Auth States
- `loading`
- `unauthenticated`
- `authenticated`

---

### 🔌 Backend Responsibilit
```

## Response Text

Created a comprehensive specification for the auth-gated docs and chatbot feature, including:
- User stories for secure content access, chatbot access, user authentication, and public content access
- Functional requirements for authentication system, database integration, content access control, chatbot access control, frontend states, and backend integration
- Non-functional requirements for security, performance, and reliability
- Success criteria with measurable outcomes
- Key entities (User, Session, Protected Content)
- Scope definition (in/out of scope)
- Assumptions and dependencies

The specification follows the template structure and is ready for the planning phase. A quality checklist was also created to validate the specification completeness.

## Outcome

Specification successfully created at specs/1-auth-gated-docs/spec.md with validation checklist at specs/1-auth-gated-docs/checklists/requirements.md. The feature branch 1-auth-gated-docs has been created and the specification is ready for planning.

## Evaluation

The specification addresses all requirements from the user description, maintains the constraint of not modifying existing backend code, and clearly defines the authentication system using Better Auth and Neon PostgreSQL. The scope is well-defined with clear boundaries between what is in and out of scope.