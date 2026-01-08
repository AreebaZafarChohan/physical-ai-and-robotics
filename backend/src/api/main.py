import logging
import json
from dotenv import load_dotenv

load_dotenv()

from fastapi import FastAPI, Request, HTTPException, Depends, WebSocket
from fastapi.responses import JSONResponse
from fastapi.websockets import WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

from backend.src.utils.logging_config import setup_logging
from backend.src.models.chat_request import ChatRequest
from backend.src.models.chat_response import ChatResponse
from backend.src.services.agent_service import AgentService
from backend.src.metrics import PrometheusMiddleware, metrics_endpoint
from backend.src.database import create_db_and_tables

from backend.src.api.auth import router as auth_router
from backend.src.api.user import router as user_router
from backend.src.api.routes.personalization import router as personalization_router
from backend.src.api.routes.user_profile import router as user_profile_router
from backend.src.api.routes.feedback import router as feedback_router
from backend.src.api.translation_router import router as translation_router

# -------------------- Setup --------------------

limiter = Limiter(key_func=get_remote_address)

setup_logging()
logger = logging.getLogger(__name__)

app = FastAPI(
    title="RAG Agent with Gemini Backend API",
    description="API for interacting with the RAG Agent, grounded by Qdrant and powered by Gemini.",
    version="1.0.0"
)

# -------------------- Startup --------------------

@app.on_event("startup")
async def on_startup():
    await create_db_and_tables()

# -------------------- CORS --------------------

ALLOWED_ORIGINS = [
    "http://localhost:3000",
    "http://localhost:3001",
    "http://127.0.0.1:3000",
    "http://127.0.0.1:3001",
    "https://physical-ai-and-robotics-five.vercel.app",
    "https://physical-ai-and-robotics-305ugn2y7-areeba-zafars-projects.vercel.app",  # Old URL (keep for compatibility)
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_origin_regex=r"https://.*areeba-zafars-projects\.vercel\.app",
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------- Middleware --------------------

app.add_middleware(PrometheusMiddleware)

app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# -------------------- Routers --------------------

app.include_router(auth_router, prefix="/auth", tags=["auth"])
app.include_router(user_router, prefix="/user", tags=["user"])
app.include_router(personalization_router, tags=["personalization"])
app.include_router(user_profile_router, tags=["user-profile"])
app.include_router(feedback_router, tags=["feedback"])
app.include_router(translation_router, prefix="", tags=["translation"])


# -------------------- Dependencies --------------------

def get_agent_service() -> AgentService:
    return AgentService()

# -------------------- REST API --------------------

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    agent_service: AgentService = Depends(get_agent_service)
):
    logger.info(f"Chat query: {request.query}")
    answer = await agent_service.get_response(
        request.query,
        request.selected_text
    )
    return ChatResponse(response=answer)

# -------------------- WebSocket --------------------

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    logger.info("WebSocket connected")

    try:
        while True:
            data = await websocket.receive_text()

            try:
                payload = json.loads(data)
                query = payload.get("query", "")
                selected_text = payload.get("selected_text", "")
            except json.JSONDecodeError:
                query = data
                selected_text = ""

            agent_service = AgentService()
            response = await agent_service.get_response(query, selected_text)

            await websocket.send_text(json.dumps({
                "query": query,
                "response": response
            }))

    except WebSocketDisconnect:
        logger.info("WebSocket disconnected")

# -------------------- Metrics --------------------

app.get("/metrics", include_in_schema=False)(metrics_endpoint)

# -------------------- Error Handling --------------------

@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    logger.error(f"{exc.status_code}: {exc.detail}")
    return JSONResponse(
        status_code=exc.status_code,
        content={"detail": exc.detail}
    )

# -------------------- Local Run --------------------

if __name__ == "__main__":
    import uvicorn
    import os

    # Set dummy environment variables for testing purposes
    os.environ["GEMINI_API_KEY"] = "test_gemini_key"
    os.environ["GEMINI_API_BASE_URL"] = "http://localhost:8080/v1"
    os.environ["QDRANT_URL"] = "http://localhost:6333"
    os.environ["QDRANT_API_KEY"] = ""
    os.environ["QDRANT_COLLECTION_NAME"] = "test_collection"
    os.environ["COHERE_API_KEY"] = "dummy_cohere_key"

    logger.info("Running FastAPI application directly with Uvicorn.")
    uvicorn.run(app, host="0.0.0.0", port=7860)