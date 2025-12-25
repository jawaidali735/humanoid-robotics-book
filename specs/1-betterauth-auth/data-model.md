# Data Model: BetterAuth Authentication System

**Feature**: User Authentication System
**Date**: 2025-12-24

## Overview

This document defines the data model for the BetterAuth authentication system. The model is based on BetterAuth's standard schema with additional considerations for the specific requirements of the project.

## Entities

### User
Represents a registered user in the system

- **id**: string (Primary Key) - Unique identifier for the user
- **email**: string (Unique) - User's email address
- **email_verified**: boolean - Whether the user's email has been verified
- **name**: string - User's display name
- **image**: string (Optional) - URL to user's profile image
- **created_at**: datetime - Timestamp when the account was created
- **updated_at**: datetime - Timestamp when the account was last updated

### Session
Manages user authentication sessions

- **id**: string (Primary Key) - Session identifier
- **user_id**: string (Foreign Key) - References the user
- **expires_at**: datetime - When this session expires
- **created_at**: datetime - When this session was created

### Account
Stores social authentication provider links

- **id**: string (Primary Key) - Account identifier
- **user_id**: string (Foreign Key) - References the user
- **provider_id**: string - Name of the provider (e.g., "github")
- **provider_user_id**: string - User ID from the provider
- **access_token**: string - Access token from the provider
- **created_at**: datetime - When this account link was created
- **updated_at**: datetime - When this account link was last updated

### VerificationToken
Handles email verification and password reset tokens

- **id**: string (Primary Key) - Token identifier
- **user_id**: string (Foreign Key) - References the user
- **token**: string - The verification token
- **expires_at**: datetime - When this token expires
- **created_at**: datetime - When this token was created

## Relationships

- User (1) ←→ (Many) Session
- User (1) ←→ (Many) Account
- User (1) ←→ (Many) VerificationToken

## Constraints

- Email must be unique across all users
- Session expiration must be in the future
- Verification tokens must expire within 24 hours of creation
- User names must not be empty
- Email addresses must follow standard email format

## Indexes

- User.email (Unique Index)
- Session.expires_at (Index for cleanup queries)
- VerificationToken.expires_at (Index for cleanup queries)
- Account.provider_id + Account.provider_user_id (Unique Index for preventing duplicate provider links)