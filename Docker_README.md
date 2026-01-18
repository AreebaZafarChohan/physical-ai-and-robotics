# Docker Setup for Physical AI & Humanoid Robotics Backend

This repository contains the backend for the Physical AI & Humanoid Robotics project, which can be containerized using Docker.

## Prerequisites

- Docker installed on your system
- Docker Compose (optional)

## Building the Docker Image

To build the Docker image for the backend, run:

```bash
docker build -t physical-ai-backend .
```

## Running the Container

To run the container:

```bash
docker run -p 9000:9000 --env-file .env physical-ai-backend
```

Make sure you have a `.env` file with all required environment variables in the root directory.

## Environment Variables

The application requires the following environment variables to be set:

```
QDRANT_URL="<YOUR_QDRANT_CLOUD_URL>"
QDRANT_API_KEY="<YOUR_QDRANT_API_KEY>"
COHERE_API_KEY="<YOUR_COHERE_API_KEY>"
GEMINI_API_KEY="<YOUR_GEMINI_API_KEY>"
BETTER_AUTH_API_KEY="<YOUR_BETTER_AUTH_API_KEY>"
BETTER_AUTH_API_SECRET="<YOUR_BETTER_AUTH_API_SECRET>"
NEON_POSTGRES_URL="<YOUR_NEON_POSTGRES_URL>"
```

## Deploying to Hugging Face Spaces

To deploy this to Hugging Face Spaces with Docker:

1. Create a Space with Docker as the SDK
2. Push this repository to the Space
3. The Dockerfile will automatically build and run the application

## Port Configuration

The application runs on port 9000 inside the container. The Dockerfile exposes this port, and you can map it to any host port when running the container.

## Troubleshooting

If you encounter issues with the build, make sure:
1. All dependencies in `backend/requirements.txt` are compatible with the Python 3.10-slim base image
2. The `.env` file contains all required environment variables
3. The application has the necessary permissions to access external services (Qdrant, Cohere, etc.)

## Development vs Production

This Dockerfile is optimized for production deployment. For development, you might want to use volume mounting to enable hot-reloading of code changes.