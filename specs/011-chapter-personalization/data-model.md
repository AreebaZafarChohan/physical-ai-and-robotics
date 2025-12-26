# Data Model: Chapter Content Personalization

## Overview
This document defines the data models required for the chapter content personalization feature. It covers user profiles, personalization rules, and related entities.

## Entity: User Profile
**Description**: Stores essential user information including their software and hardware background collected during signup

**Fields**:
- `id`: int, primary key, auto-increment
- `user_id`: int, foreign key to users table
- `software_background`: JSON array of strings (e.g., ["Python", "JavaScript"])
- `hardware_background`: JSON array of strings (e.g., ["Raspberry Pi", "Arduino"])
- `experience_level`: string enum ["beginner", "intermediate", "advanced"]
- `created_at`: datetime, timestamp when record was created
- `updated_at`: datetime, timestamp when record was last updated

## Entity: Personalization Rules
**Description**: Defined mappings between user profile attributes and content variations, stored in a dedicated database table for efficient retrieval and scalability

**Fields**:
- `id`: int, primary key, auto-increment
- `content_path`: string, the path of content this rule applies to (e.g., "/module-1/chapter-1")
- `user_profile_criteria`: JSONB, matching criteria for user profiles (e.g., {"software_background": ["Python"], "experience_level": "beginner"})
- `content_variants`: JSONB, different content variants keyed by profile type (e.g., {"beginner": "content", "python-focused": "content"})
- `description`: text, optional description of what this rule does
- `priority`: int, priority for rule evaluation when multiple rules match (lower numbers = higher priority)
- `is_active`: boolean, whether this rule is currently active
- `created_at`: datetime, timestamp when rule was created
- `updated_at`: datetime, timestamp when rule was last updated

## Entity: Personalization Cache
**Description**: Cache table to store pre-computed personalized content to improve performance

**Fields**:
- `id`: int, primary key, auto-increment
- `user_profile_hash`: string, hashed combination of user profile attributes
- `content_path`: string, path of cached content
- `personalized_content`: text, the fully processed personalized content
- `cache_created_at`: datetime, timestamp when cache entry was created
- `cache_expires_at`: datetime, timestamp when cache entry expires (24 hours after creation)

## Entity: Personalization Markers
**Description**: Registry of all personalization markers found in content (optional, for management purposes)

**Fields**:
- `id`: int, primary key, auto-increment
- `content_path`: string, path of content containing the marker
- `marker_type`: string, the type of marker (e.g., "beginner", "python-focused")
- `marker_location`: text, exact location/coordinates in content (for editor tools)
- `description`: text, optional description of what the marker does

## Relationships
- User Profile has a one-to-many relationship with Personalization Cache (one user profile may have multiple cached personalized content versions)
- Personalization Rules are applied to content based on matching User Profile criteria

## Validation Rules
Based on functional requirements from the spec:

1. **FR-006**: User Profile data must include software_background and hardware_background from signup
2. **FR-007**: Personalization Rules must be rule-based (not full AI-generated), which is enforced by the structured schema
3. **FR-004.1**: Cache entries must have expiration times (24 hours) for invalidation

## State Transitions
- Personalization Rules: Active ↔ Inactive (via is_active flag)
- User Profile: Created → Updated (when user profile changes trigger re-personalization)