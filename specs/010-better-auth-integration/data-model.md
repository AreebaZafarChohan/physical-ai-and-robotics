# Data Model for Signup & Signin Integration

**Date**: 2025-12-22
**Spec**: specs/010-better-auth-integration/spec.md
**Plan**: specs/010-better-auth-integration/plan.md

## Entities

### User

**Description**: Represents a registered individual within the RAG chatbot book application.

**Attributes**:

-   `id`: (string, unique, primary key) - Unique identifier for the user.
-   `email`: (string, unique, required) - User's email address.
-   `password_hash`: (string, required, securely stored) - Hashed password for email/password authentication.
-   `oauth_provider_ids`: (object, optional) - Stores unique identifiers from OAuth providers if the user registers/logs in via OAuth.
    -   Example: `{"google": "google_id_string", "github": "github_id_string"}`
-   `software_background`: (array of strings, optional) - Information about the user's software expertise (e.g., programming languages, operating systems, tools). Allows predefined categories with an "other" free-text option.
-   `hardware_background`: (array of strings, optional) - Information about the user's hardware familiarity (e.g., specific components, robot types). Allows predefined categories with an "other" free-text option.
-   `session_token`: (string, securely stored, volatile) - Token used to maintain user session state after successful login.

**Relationships**:

-   `PersonalizationData`: One-to-one relationship with `User`. Stores more detailed software/hardware preferences and other data used for content personalization.

**Validation Rules**:

-   `email`: Must be a valid email format, unique across all users.
-   `password_hash`: Must correspond to a password adhering to the policy: Minimum 8 characters, at least one uppercase, one lowercase, one number.
-   `software_background`, `hardware_background`: Each string in the array should conform to either a predefined list or reasonable free-text length limits.
