# Data Model: User Authentication

This document defines the data models for the user authentication feature.

## User

Represents a user of the website.

**Fields**:

- `id` (string, primary key): A unique identifier for the user. This will be the ID from the Better-Auth service.
- `email` (string, unique): The user's email address.
- `software_experience` (string): The user's self-reported software experience.
- `hardware_experience` (string): The user's self-reported hardware experience.
- `created_at` (timestamp): The timestamp when the user account was created.
- `updated_at` (timestamp): The timestamp when the user account was last updated.

**Relationships**:

- A user can have one session at a time.

**Validation Rules**:

- `email` must be a valid email format.
- `software_experience` and `hardware_experience` are free-form text fields.

## UserSession

Represents an authenticated user session.

**Fields**:

- `session_id` (string, primary key): A unique identifier for the session, provided by Better-Auth.
- `user_id` (string, foreign key): The ID of the user associated with this session.
- `expires_at` (timestamp): The timestamp when the session expires.

**Relationships**:

- A session belongs to one user.
