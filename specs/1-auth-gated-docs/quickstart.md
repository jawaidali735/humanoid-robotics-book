# Quickstart Guide: Auth-Gated Docs & Chatbot

**Feature**: Auth-Gated Docs & Chatbot
**Created**: 2025-12-23
**Status**: Draft

## Overview
This guide provides a quick setup and development workflow for the authentication system in the Auth-Gated Docs & Chatbot feature.

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Neon PostgreSQL account and database
- Better Auth account (or local setup)

## Environment Setup

### 1. Clone and Install Dependencies
```bash
git clone <repository-url>
cd <repository-name>
npm install
```

### 2. Environment Variables
Create a `.env` file in the project root with the following variables:

```env
# Better Auth Configuration
AUTH_SECRET=your-auth-secret-here
AUTH_URL=http://localhost:8000

# Neon PostgreSQL Configuration
DATABASE_URL=your-neon-database-url

# Application Configuration
NEXTAUTH_URL=http://localhost:3000
```

### 3. Database Setup
The system uses Neon PostgreSQL with the following tables:
- `users` - stores user account information
- `sessions` - stores active session data

Better Auth will handle table creation if configured properly.

## Development Workflow

### 1. Start the Development Server
```bash
# Start the Docusaurus development server
npm run start

# In a separate terminal, start the backend auth server
cd backend/auth
npm run dev
```

### 2. Running Tests
```bash
# Run authentication unit tests
npm run test:auth

# Run integration tests
npm run test:integration

# Run all tests
npm run test:all
```

## Key Files and Directories

### Backend Auth Module
```
backend/auth/
├── index.js              # Main auth server entry point
├── config.js             # Better Auth configuration
├── models/               # Database models
│   ├── user.js
│   └── session.js
├── routes/               # Authentication routes
│   ├── auth.js
│   ├── session.js
│   └── validation.js
├── middleware/           # Auth middleware
│   └── auth-guard.js
└── utils/                # Utility functions
    └── validators.js
```

### Frontend Integration
```
src/components/
├── Auth/
│   ├── AuthProvider.js      # Authentication context provider
│   ├── LoginForm.js         # Login form component
│   ├── RegisterForm.js      # Registration form component
│   └── ProtectedRoute.js    # Protected route wrapper
└── Content/
    ├── ContentGate.js       # Content gating component
    └── Chatbot/
        └── AuthenticatedChatbot.js
```

## API Endpoints

### Authentication Endpoints
- `POST /api/auth/register` - User registration
- `POST /api/auth/login` - User login
- `POST /api/auth/logout` - User logout
- `GET /api/auth/session` - Get current session

### Content Access Endpoints
- `POST /api/auth/content-access` - Check content access
- `POST /api/auth/chat-access` - Check chat access

## Common Development Tasks

### 1. Adding a New Protected Route
```javascript
// Example: Protecting a new content route
import { withAuth } from '../middleware/auth-guard';

const ProtectedContent = () => {
  return <div>Protected content here</div>;
};

export default withAuth(ProtectedContent);
```

### 2. Checking Authentication in Components
```javascript
// Example: Checking auth state in a component
import { useAuth } from '../components/Auth/AuthProvider';

const MyComponent = () => {
  const { user, isAuthenticated, loading } = useAuth();

  if (loading) return <div>Loading...</div>;
  if (!isAuthenticated) return <div>Please log in</div>;

  return <div>Content for authenticated users</div>;
};
```

### 3. Adding Custom Validation
```javascript
// Example: Adding custom validation to registration
import { validateEmail, validatePassword } from '../utils/validators';

const registerUser = async (userData) => {
  const emailValidation = validateEmail(userData.email);
  const passwordValidation = validatePassword(userData.password);

  if (!emailValidation.isValid) {
    throw new Error(emailValidation.message);
  }

  // Process registration...
};
```

## Testing Authentication

### Unit Tests
```javascript
// Example test for login functionality
import { login } from '../auth/login';

test('successful login returns user data', async () => {
  const result = await login('test@example.com', 'password123');
  expect(result.success).toBe(true);
  expect(result.user).toBeDefined();
});
```

### Integration Tests
```javascript
// Example integration test
import request from 'supertest';
import app from '../app';

test('protected endpoint requires authentication', async () => {
  const response = await request(app)
    .get('/api/protected-content')
    .expect(401);

  expect(response.body.message).toBe('Authentication required');
});
```

## Troubleshooting

### Common Issues

1. **Session not persisting across page refresh**
   - Check that HttpOnly cookies are properly configured
   - Verify AUTH_SECRET is consistent across environments

2. **Database connection errors**
   - Confirm DATABASE_URL is correctly set
   - Check Neon PostgreSQL connection settings

3. **Authentication state not updating**
   - Ensure AuthProvider is wrapped around the correct component tree
   - Verify session endpoints are properly configured

### Debugging Tips
- Enable detailed logging in development: `DEBUG=auth:* npm run dev`
- Check browser network tab for authentication requests
- Use Better Auth's built-in debugging tools

## Deployment

### Environment Variables for Production
```env
# Production-specific variables
NODE_ENV=production
AUTH_SECRET=production-auth-secret
DATABASE_URL=production-neon-database-url
NEXTAUTH_URL=https://yourdomain.com
```

### Deployment Steps
1. Build the frontend: `npm run build`
2. Build the backend auth module: `cd backend/auth && npm run build`
3. Deploy both to your hosting platform
4. Set production environment variables
5. Verify authentication endpoints are accessible