---
title: "Data Model: Qdrant Embedding Pipeline"
description: "Data model for the vector store."
date: 2025-12-09
feature: "[[003-qdrant-ingestion-pipeline]]"
branch: "[[003-qdrant-ingestion-pipeline]]"
---

## 1. Overview

The data model for this feature is centered on the structure of the documents to be stored in the Qdrant vector database. Each document, or "point" in Qdrant's terminology, consists of a vector embedding and a JSON payload.

## 2. Qdrant Collection

- **Collection Name**: `ai_robotics_book`

### 2.1. Vector Parameters

- **Size**: 1024 (determined by Cohere's `embed-english-v3.0` model)
- **Distance Metric**: `COSINE`

### 2.2. Point Structure

Each point in the collection will have the following structure:

- **`id`** (UUID/integer): A unique identifier for the point. A simple incrementing integer is sufficient for this use case.
- **`vector`** (array of floats): The 1024-dimensional embedding of the text chunk.
- **`payload`** (JSON object): Metadata associated with the vector.

#### Payload Schema

| Field      | Type   | Description                                            | Example                                         |
|------------|--------|--------------------------------------------------------|-------------------------------------------------|
| `url`      | string | The source URL from which the text was extracted.      | `"https://.../intro-to-ros2"`                   |
| `text`     | string | The raw text chunk that was embedded.                  | `"ROS 2 is a set of software libraries..."`     |
| `chunk_id` | integer| The sequential index of the chunk within its source URL. | `1`                                             |
