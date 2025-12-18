---
title: "Research: Qdrant Embedding Pipeline"
description: "Research and decisions made for the ingestion pipeline."
date: 2025-12-09
feature: "[[003-qdrant-ingestion-pipeline]]"
branch: "[[003-qdrant-ingestion-pipeline]]"
---

## 1. Research Summary

The implementation plan for the Qdrant ingestion pipeline relies on a well-defined and standard stack, as outlined in the project constitution and feature specification. All technical choices were provided, and no significant ambiguities required further research.

### Key Decisions (Pre-defined)

- **Vector Database**: Qdrant Cloud was chosen for its managed service, scalability, and Python client library.
- **Embedding Model**: Cohere's `embed-english-v3.0` was selected for its performance and because it is a standard choice for English-language embedding tasks. The vector size of 1024 is determined by this model.
- **Text Extraction**: `trafilatura` is a robust library specifically designed for web content extraction, making it a suitable choice over manual parsing or other generic libraries.
- **Distance Metric**: COSINE similarity is the standard and most effective distance metric for comparing the semantic similarity of text embeddings.
