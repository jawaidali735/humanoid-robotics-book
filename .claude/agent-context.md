# Claude Agent Context: Auth-Gated Docs & Chatbot

## Feature Overview
- **Feature**: Auth-Gated Docs & Chatbot
- **Branch**: 1-auth-gated-docs
- **Purpose**: Implement secure authentication system for Docusaurus-based book website with attached chatbot
- **Scope**: User authentication, content access control, chatbot access control

## Technical Architecture
- **Authentication Provider**: Better Auth
- **Database**: Neon PostgreSQL
- **Session Management**: HttpOnly cookies with JWT
- **Frontend**: Docusaurus with React components
- **Backend**: Isolated `backend/auth/` module

## Key Components
1. **User Management**: Registration, login, logout
2. **Session Handling**: Creation, validation, expiration
3. **Content Gating**: Protected book chapters with blur/overlay
4. **Chatbot Control**: Access validation and user tracking
5. **Frontend State**: Loading, unauthenticated, authenticated

## Database Schema
### Users Table
- id (uuid, PK)
- email (unique)
- name
- password_hash
- role (guest/user/admin)
- created_at

### Sessions Table
- id (uuid)
- user_id (FK)
- expires_at

## API Endpoints
- POST /api/auth/register
- POST /api/auth/login
- POST /api/auth/logout
- GET /api/auth/session
- POST /api/auth/content-access
- POST /api/auth/chat-access

## Constraints
- Existing backend code must remain untouched
- All auth logic in isolated `backend/auth/` folder
- Integration with existing RAG, embeddings, and chatbot without refactoring
- Content access control for book chapters
- Chatbot access validation

## Security Considerations
- Password hashing with bcrypt
- HttpOnly cookies for session tokens
- Input validation on all endpoints
- Rate limiting for auth endpoints
- Proper error handling without information disclosure

## Frontend Integration
- React Context for auth state management
- Conditional rendering based on auth status
- Protected route components
- Content gating with blur/overlay effect
- Chatbot access control

## Development Notes
- Environment variables: AUTH_SECRET, DATABASE_URL, NEXTAUTH_URL
- Neon PostgreSQL connection configuration
- Better Auth setup and configuration
- Docusaurus custom component integration