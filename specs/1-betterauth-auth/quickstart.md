# Quickstart Guide: BetterAuth Authentication

**Feature**: User Authentication System
**Date**: 2025-12-24

## Overview

This guide provides a quick setup for implementing BetterAuth authentication in your application with Neon database and Drizzle ORM.

## Prerequisites

- Node.js 18+ installed
- Neon database account and connection string
- Basic understanding of your frontend framework

## Installation

1. Install BetterAuth and dependencies:
```bash
npm install better-auth
npm install better-auth/adapters/drizzle
npm install drizzle-orm
# Install your database driver (e.g., for PostgreSQL)
npm install @neondatabase/serverless
```

2. Install CLI tools:
```bash
npm install -D @better-auth/cli
```

## Backend Setup

1. Create a `.env` file with required environment variables:
```env
BETTER_AUTH_SECRET=your-super-secret-key-here-32-characters-minimum
BETTER_AUTH_URL=http://localhost:3000
DATABASE_URL=your-neon-database-connection-string
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
```

2. Create `auth.ts` in your project:
```typescript
import { betterAuth } from "better-auth";
import { drizzleAdapter } from "better-auth/adapters/drizzle";
import { db } from "./db"; // your drizzle instance

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: "pg", // for PostgreSQL/Neon
  }),
  emailAndPassword: {
    enabled: true,
    autoSignIn: true,
  },
  socialProviders: {
    github: {
      clientId: process.env.GITHUB_CLIENT_ID!,
      clientSecret: process.env.GITHUB_CLIENT_SECRET!,
    },
  },
});
```

3. Create the authentication route handler (for Next.js App Router):
```typescript
// app/api/auth/[...all]/route.ts
import { auth } from "@/lib/auth"; // path to your auth file
import { toNextJsHandler } from "better-auth/next-js";
export const { POST, GET } = toNextJsHandler(auth);
```

## Database Setup

1. Generate the database schema:
```bash
npx @better-auth/cli generate
```

2. Run the migration:
```bash
npx @better-auth/cli migrate
```

## Frontend Setup

1. Create a client authentication instance:
```typescript
// lib/auth-client.ts
import { createAuthClient } from "better-auth/react"
export const authClient = createAuthClient({
    baseURL: "http://localhost:3000" // or your production URL
})
```

2. Use in your components:
```typescript
// Example signup component
import { authClient } from "@/lib/auth-client";

const handleSignUp = async (email: string, password: string, name: string) => {
  const { data, error } = await authClient.signUp.email({
    email,
    password,
    name,
  });

  if (error) {
    console.error("Signup error:", error.message);
    return;
  }

  // Handle successful signup
  console.log("User created:", data?.user);
};
```

## Next Steps

1. Implement sign-in and sign-up forms
2. Add session management with useSession hook
3. Create protected routes/components
4. Add GitHub social login functionality
5. Implement logout functionality