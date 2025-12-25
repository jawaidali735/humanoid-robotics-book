# Agent Context: BetterAuth Authentication Implementation

## Feature Overview
- Feature: User Authentication System with BetterAuth
- Branch: 1-betterauth-auth
- Status: Implementation in progress

## Technology Stack
- Authentication: BetterAuth library
- Database: Neon database with Drizzle ORM
- Backend: Node.js
- Frontend: React components (or vanilla JavaScript)
- Environment: Node.js runtime

## Key Components

### Backend
- `auth.ts` - Main BetterAuth configuration
- Database adapter for Neon/Drizzle
- Email/password authentication
- GitHub social authentication
- API routes at `/api/auth/*`

### Frontend
- `auth-client.ts` - Client authentication instance
- Sign-up form component
- Login form component
- Session management hooks
- Protected route components

## Database Schema
- User table (id, email, name, image, timestamps)
- Session table (session management)
- Account table (social auth links)
- VerificationToken table (email verification, password reset)

## Environment Variables
- `BETTER_AUTH_SECRET` - Secret key (32+ chars)
- `BETTER_AUTH_URL` - Base URL for app
- `GITHUB_CLIENT_ID` - GitHub OAuth client ID
- `GITHUB_CLIENT_SECRET` - GitHub OAuth client secret
- `DATABASE_URL` - Neon database connection string

## API Endpoints
- POST `/api/auth/sign-up` - User registration
- POST `/api/auth/sign-in` - User login
- POST `/api/auth/sign-out` - User logout
- GET `/api/auth/session` - Get current session
- POST `/api/auth/sign-in/github` - GitHub social login

## Implementation Steps
1. Backend setup with database integration
2. Frontend authentication components
3. Social authentication (GitHub)
4. Session management
5. Security implementation

## Security Considerations
- Secure secret key management
- HTTPS in production
- Input validation
- Rate limiting for auth endpoints
- Secure session handling

## CLI Commands
- `npx @better-auth/cli generate` - Generate schema
- `npx @better-auth/cli migrate` - Run migrations