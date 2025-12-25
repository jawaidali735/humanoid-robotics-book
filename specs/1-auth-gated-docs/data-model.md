# Data Model: Auth-Gated Docs & Chatbot

**Feature**: Auth-Gated Docs & Chatbot
**Created**: 2025-12-23
**Status**: Final

## Entity: User

### Fields
- **id** (uuid, primary key)
  - Type: UUID
  - Required: Yes
  - Unique: Yes
  - Description: Unique identifier for the user

- **email** (string)
  - Type: VARCHAR(255)
  - Required: Yes
  - Unique: Yes
  - Validation: Must be valid email format
  - Description: User's email address for login

- **name** (string)
  - Type: VARCHAR(255)
  - Required: No
  - Description: User's display name

- **password_hash** (string)
  - Type: TEXT
  - Required: Yes
  - Description: Hashed password using bcrypt or similar

- **role** (string)
  - Type: VARCHAR(50)
  - Required: Yes
  - Default: 'user'
  - Values: 'guest' | 'user' | 'admin'
  - Description: User's access level

- **created_at** (timestamp)
  - Type: TIMESTAMP
  - Required: Yes
  - Default: Current timestamp
  - Description: Account creation time

### Relationships
- One User has many Sessions (via user_id foreign key)

### Validation Rules
- Email must be unique
- Email must follow valid email format
- Password must be hashed (never stored in plain text)
- Role must be one of allowed values

## Entity: Session

### Fields
- **id** (uuid, primary key)
  - Type: UUID
  - Required: Yes
  - Unique: Yes
  - Description: Unique identifier for the session

- **user_id** (uuid, foreign key)
  - Type: UUID
  - Required: Yes
  - References: User.id
  - Description: Links session to a user

- **expires_at** (timestamp)
  - Type: TIMESTAMP
  - Required: Yes
  - Description: Session expiration time

- **created_at** (timestamp)
  - Type: TIMESTAMP
  - Required: Yes
  - Default: Current timestamp
  - Description: Session creation time

### Relationships
- One Session belongs to one User (via user_id foreign key)

### Validation Rules
- user_id must reference an existing User
- expires_at must be in the future
- Session must be invalidated after expiration

## Entity: ProtectedContentAccess (Conceptual)

### Description
This is a conceptual entity representing the relationship between users and protected content access. This is handled through authentication checks rather than a physical database table.

### Fields (Conceptual)
- **user_id** (uuid, foreign key)
  - References: User.id
  - Description: User requesting access

- **content_path** (string)
  - Type: VARCHAR(500)
  - Description: Path to protected content

- **access_granted** (boolean)
  - Type: BOOLEAN
  - Default: false
  - Description: Whether access was granted

### Validation Rules (Conceptual)
- Access granted only if user is authenticated
- Access logs may be maintained for analytics

## Database Constraints

### User Table
- UNIQUE constraint on email field
- NOT NULL constraints on required fields
- CHECK constraint on role field values

### Session Table
- FOREIGN KEY constraint from user_id to User.id
- NOT NULL constraints on required fields
- CHECK constraint ensuring expires_at is in the future

## Indexes

### User Table
- Primary index on id
- Unique index on email

### Session Table
- Primary index on id
- Foreign key index on user_id
- Index on expires_at for efficient cleanup

## State Transitions

### User Role Transitions
- guest → user (upon registration)
- user → admin (via admin action)
- admin → user (via admin action)

### Session State Transitions
- Active (when created)
- Expired (when expires_at is reached)
- Invalidated (when explicitly logged out)