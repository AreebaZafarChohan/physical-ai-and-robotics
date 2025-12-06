# Data Model: Physical AI & Humanoid Robotics Textbook

**Date**: 2025-12-06
**Feature**: Physical AI & Humanoid Robotics Textbook

This document outlines the data model for the textbook. As the project is a static site generated from Markdown files, the "data model" primarily refers to the content structure and metadata.

## Entities

### Book

Represents the entire textbook. It is defined by the `docusaurus.config.js` file and the overall directory structure.

-   **Attributes**:
    -   `title` (string): The title of the book.
    -   `author` (string): The author's name.
    -   `description` (string): A brief description of the book.
    -   `version` (string): The book's version number.
-   **Relationships**:
    -   Has many `Modules`.

### Module

A module is a top-level section of the book, represented by a subdirectory within the `/docs` directory.

-   **Attributes**:
    -   `name` (string): The name of the module (e.g., "The Robotic Nervous System (ROS 2)").
    -   `order` (integer): The order in which the module appears in the sidebar.
-   **Relationships**:
    -   Belongs to a `Book`.
    -   Has many `Chapters`.

### Chapter

A chapter is an individual Markdown file within a module's subdirectory.

-   **Attributes**:
    -   `title` (string): The title of the chapter.
    -   `slug` (string): The URL-friendly identifier for the chapter.
    -   `content` (Markdown): The main content of the chapter.
-   **Relationships**:
    -   Belongs to a `Module`.
    -   Contains many `Content Blocks`.

### Content Block

A content block is an element within a chapter's Markdown file.

-   **Types**:
    -   `Text`: Paragraphs, lists, etc.
    -   `Code Snippet`: Formatted code examples.
    -   `Diagram`: Images or illustrations.
    -   `Placeholder`: Markers for interactive elements like `[RAG Chatbot]`.

## State Transitions

-   The content is version-controlled via Git. The state of the book (e.g., `draft`, `published`) is managed through branches and deployments. The `main` branch represents the published version.
