# Quickstart Guide for AI-Native RAG Chatbot System

This guide provides instructions to quickly set up and run the AI-Native RAG Chatbot system.

## 1. Prerequisites

Before you begin, ensure you have the following installed:

-   **Python 3.10+**
-   **Node.js v18+**
-   **npm** or **yarn**
-   **Docker** (for local Qdrant/Neon setup if not using cloud services)
-   **Git**

## 2. Clone the Repository

```bash
git clone https://github.com/your-org/Physical_AI_And_Robotics.git
cd Physical_AI_And_Robotics
git checkout 001-rag-chatbot-integration
```

## 3. Environment Setup

Create a `.env` file in the `backend/` directory based on `backend/.env.example`.

```ini
# backend/.env
QDRANT_API_KEY="YOUR_QDRANT_CLOUD_API_KEY"
QDRANT_URL="YOUR_QDRANT_CLOUD_URL" # e.g., "https://<cluster-id>.qdrant.tech:6333"
NEON_DATABASE_URL="YOUR_NEON_POSTGRES_CONNECTION_STRING" # e.g., "postgresql://user:password@host:port/database"
OPENAI_API_KEY="YOUR_GEMINI_VIA_OPENAI_WRAPPER_API_KEY" # Key for OpenAI-compatible endpoint for Gemini
JWT_SECRET_KEY="YOUR_STRONG_RANDOM_SECRET_KEY"
ALGORITHM="HS256"
ACCESS_TOKEN_EXPIRE_MINUTES=30
FRONTEND_ORIGIN="http://localhost:3000" # Or your Docusaurus deployment URL
```
**Note**: Ensure `QDRANT_API_KEY` is a read-only key.

## 4. Backend Setup (FastAPI)

```bash
cd backend
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt # Create requirements.txt from plan.md
python main.py
```
The backend will run on `http://localhost:8000`.

## 5. Frontend Setup (Docusaurus with ChatKit)

```bash
cd frontend
npm install # or yarn install
npm start # or yarn start
```
The Docusaurus frontend will typically run on `http://localhost:3000`.

## 6. Access the Chatbot

Once both backend and frontend are running, open your web browser and navigate to `http://localhost:3000`. The floating chatbot UI should be visible. You can interact with it, sign up/in, and test both RAG modes.
