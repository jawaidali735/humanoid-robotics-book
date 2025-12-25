---
id: 2
title: Production RAG Backend Plan
stage: plan
date: 2025-12-22
surface: agent
model: claude-sonnet-4-5-20250929
feature: production-rag-backend
branch: 001-production-rag-backend
user: Jawaid Ali
command: /sp.plan
labels: ["plan", "rag", "backend", "fastapi", "agents", "qdrant", "architecture"]
links:
  spec: specs/001-production-rag-backend/spec.md
  plan: specs/001-production-rag-backend/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-production-rag-backend/plan.md
  - specs/001-production-rag-backend/research.md
  - specs/001-production-rag-backend/data-model.md
  - specs/001-production-rag-backend/quickstart.md
  - specs/001-production-rag-backend/contracts/chat-api.yaml
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

Production RAG Backend with FastAPI + OpenAI Agents SDK + Qdrant + Gemini

Project Goal:
Build a production-ready RAG (Retrieval-Augmented Generation) chatbot backend that:
- Receives user questions from a Docusaurus frontend via FastAPI
- Uses the official OpenAI Agents SDK to create a QA (Question-Answering) agent
- Allows the agent to call a retrieval tool to fetch relevant book content from Qdrant
- Generates accurate, grounded answers based only on retrieved content
- Returns JSON responses to the frontend

Key Constraints:
- Existing folders data_embedding/ and data_retrieval/ MUST NOT be modified in any way
- Only use official OpenAI Agents SDK syntax (Agent, Runner, @function_tool, etc.) — no LangChain, no other frameworks
- Use Gemini model via custom AsyncOpenAI client with Google endpoint
- All FastAPI endpoints must be asynchronous
- Code must be clean, readable, production-ready, and JSON-safe

Technologies:
- FastAPI (API layer)
- OpenAI Agents SDK (Agent + Runner + Tools)
- Qdrant (vector database — accessed via existing data_retrieval/ functions)
- Gemini API (via OpenAI Agents SDK custom client)
- Python (async)

Folder Structure to Follow:
backend/
├── app/
│   ├── main.py
│   ├── api/
│   │   └── chat.py
│   ├── agents/
│   │   ├── qa_agent.py
│   │   └── tools.py
│   ├── config/
│   │   └── setting.py (already exists)
│   └── schemas/
│       └── chat.py (to be created)
├── data_embedding/ (existing — do not touch)
└── data_retrieval/ (existing — do not touch)

High-Level Components to Plan:
1. FastAPI application setup in main.py
2. Pydantic schemas for request and response
3. Configuration loading (API keys, Qdrant settings) from config/setting.py
4. Retrieval tool in agents/tools.py that calls existing functions from data_retrieval/
5. QA Agent definition in agents/qa_agent.py with clear system instructions and the retrieval tool
6. Gemini model integration using the exact custom client pattern provided
7. Async chat endpoint in api/chat.py that runs the agent and returns final_output
8. Basic error handling and logging

Success Criteria:
- Plan clearly defines every new file and its exact purpose
- Specifies file paths inside app/
- Describes the full flow: Frontend → FastAPI endpoint → Agent → Retrieval Tool → Qdrant → Agent → JSON Response
- Includes rough structure for:
  - Agent name, instructions, and tools
  - Retrieval tool signature
  - Gemini client/model configuration
  - Async agent execution in FastAPI
- Ensures zero modification to existing folders
- Makes it easy to create /sp.specify prompts for each file later

Timeline:
- This plan should be finalized within 1 day
- Actual code implementation in 3–5 days

Not Planning in Detail Yet:
- Line-by-line code implementation
- Advanced agent features (handoffs, guardrails, sessions) — keep it simple for v1
- Authentication, rate limiting, or deployment details

Output Format:
A clean, structured markdown pl

## Response snapshot

Completed comprehensive planning for Production RAG Backend with FastAPI, OpenAI Agents SDK, and Qdrant integration, including implementation plan, research findings, data models, API contracts, and quickstart guide.

## Outcome

- ✅ Impact: Complete implementation plan with all required artifacts created for RAG backend development
- 🧪 Tests: none
- 📁 Files: specs/001-production-rag-backend/plan.md, research.md, data-model.md, quickstart.md, contracts/chat-api.yaml, updated CLAUDE.md
- 🔁 Next prompts: /sp.tasks to break down implementation into testable tasks
- 🧠 Reflection: Clear separation maintained between new components and existing data directories as required

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A