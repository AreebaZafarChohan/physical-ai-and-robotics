---
title: "Quickstart: Qdrant Embedding Pipeline"
description: "How to run the ingestion script."
date: 2025-12-09
feature: "[[003-qdrant-ingestion-pipeline]]"
branch: "[[003-qdrant-ingestion-pipeline]]"
---

## 1. Overview

This guide provides the steps to run the `ingest_book.py` script. This script will populate your Qdrant Cloud database with the content of the "Physical AI & Humanoid Robotics" textbook.

## 2. Prerequisites

- Python 3.10 or higher.
- A Qdrant Cloud account and API key.
- A Cohere account and API key.

## 3. Setup

### 3.1. Environment Variables

Create a `.env` file in the same directory as the script, or set the following environment variables in your system:

```
QDRANT_URL="<YOUR_QDRANT_CLOUD_URL>"
QDRANT_API_KEY="<YOUR_QDRANT_API_KEY>"
COHERE_API_KEY="<YOUR_COHERE_API_KEY>"
```

### 3.2. Install Dependencies

Install the required Python libraries:

```bash
pip install requests "qdrant-client[fastembed]" cohere trafilatura
```

## 4. Running the Script

Execute the script from your terminal:

```bash
python ingest_book.py
```

### 4.1. Expected Output

The script will provide logs indicating its progress. You should see output similar to the following:

```
INFO:root:Found 25 URLs in the sitemap.
INFO:root:Processing URL 1/25: https://physical-ai-and-robotics-five.vercel.app/docs/intro
INFO:root:Extracted 5 chunks from the URL.
INFO:root:Upserted 5 points to Qdrant collection 'ai_robotics_book'.
...
INFO:root:Processing URL 25/25: https://physical-ai-and-robotics-five.vercel.app/docs/capstone-project/autonomous-humanoid
...
INFO:root:Successfully ingested 150 chunks into the Qdrant collection.
INFO:root:Validation complete: Found 150 points in the collection.
```

## 5. Verification

After the script completes, you can log in to your Qdrant Cloud dashboard to verify that the `ai_robotics_book` collection has been created and populated with the correct number of points.
