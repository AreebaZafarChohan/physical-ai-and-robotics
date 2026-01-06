import logging
import json
from dotenv import load_dotenv
load_dotenv()
from fastapi.responses import JSONResponse
from fastapi import FastAPI, Request, HTTPException, Depends
from fastapi import WebSocket
from fastapi.websockets import WebSocketDisconnect
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

from backend.src.utils.logging_config import setup_logging
from backend.src.models.chat_request import ChatRequest
from backend.src.models.chat_response import ChatResponse
from backend.src.services.agent_service import AgentService
from backend.src.metrics import PrometheusMiddleware, metrics_endpoint
from backend.src.database import create_db_and_tables # New import
from backend.src.api.auth import router as auth_router # New import
from backend.src.api.user import router as user_router # New import
# from backend.src.api.routes.personalization import router as personalization_router # Personalization routes
# from backend.src.api.routes.user_profile import router as user_profile_router # User profile routes
# from backend.src.api.routes.feedback import router as feedback_router # Feedback routes
# # import router as translation_router # New import for translation API
# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Setup logging as early as possible
setup_logging()
logger = logging.getLogger(__name__)

from fastapi.middleware.cors import CORSMiddleware

# ... existing imports ...

app = FastAPI(
    title="RAG Agent with Gemini Backend API",
    description="API for interacting with the RAG Agent, grounded by Qdrant and powered by Gemini.",
    version="1.0.0"
)

@app.on_event("startup")
async def on_startup():
    await create_db_and_tables()

# CORS allowed origins - localhost for dev + Vercel for production
ALLOWED_ORIGINS = [
    # Development - localhost
    "http://localhost:3000",
    "http://localhost:3001",
    "http://127.0.0.1:3000",
    "http://127.0.0.1:3001",
    # Vercel Production deployment - UPDATED URL
    "https://physical-ai-and-robotics-five.vercel.app",
    "https://physical-ai-and-robotics-305ugn2y7-areeba-zafars-projects.vercel.app",  # Old URL (keep for compatibility)
]

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_origin_regex=r"https://.*areeba-zafars-projects\.vercel\.app",  # Allow all Vercel preview deployments
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add Prometheus middleware
app.add_middleware(PrometheusMiddleware)

# Add rate limiting
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Include authentication router
app.include_router(auth_router, prefix="/auth", tags=["auth"])

# Include user router
app.include_router(user_router, prefix="/user", tags=["user"])
                                                                                                                                    # Include translation router
# app.include_router(translation_router, prefix="", tags=["translation"])
# # Include personalization router
# app.include_router(personalization_router, tags=["personalization"])

# # Include user profile router
# app.include_router(user_profile_router, tags=["user-profile"])

# # Include feedback router
# app.include_router(feedback_router, tags=["feedback"])

def get_agent_service() -> AgentService:
    """Dependency injector for AgentService."""
    return AgentService()

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    agent_service: AgentService = Depends(get_agent_service)
):
    """
    Endpoint to send a query to the RAG Agent and get a grounded response.
    """
    logger.info(f"Received /chat request with query: '{request.query}' and selected_text: '{request.selected_text}'")
    answer = await agent_service.get_response(request.query, request.selected_text)
    return ChatResponse(response=answer)

# Add WebSocket endpoint for real-time communication
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    logger.info("WebSocket connection established")
    try:
        while True:
            data = await websocket.receive_text()
            # Parse the received data as JSON
            import json
            try:
                message_data = json.loads(data)
                query = message_data.get("query", "")
                selected_text = message_data.get("selected_text", "")
            except json.JSONDecodeError:
                # If it's not JSON, treat the whole message as a query
                query = data
                selected_text = ""

            # Get response from agent service
            agent_service = AgentService()
            response = await agent_service.get_response(query, selected_text)

            # Send response back to client
            response_data = {
                "response": response,
                "query": query
            }
            await websocket.send_text(json.dumps(response_data))
    except WebSocketDisconnect:
        logger.info("WebSocket connection closed")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        await websocket.close()


# Add metrics endpoint
app.get("/metrics", include_in_schema=False)(metrics_endpoint)


@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    """
    Custom exception handler for HTTPException to ensure consistent error responses.
    """
    logger.error(f"HTTPException caught: Status {exc.status_code}, Detail: {exc.detail}")
    return JSONResponse(
        status_code=exc.status_code,
        content={"detail": exc.detail}
    )

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
    uvicorn.run(app, host="0.0.0.0", port=9000)