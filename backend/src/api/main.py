import logging
import json
from dotenv import load_dotenv
load_dotenv()
from fastapi.responses import JSONResponse
from fastapi import FastAPI, Request, HTTPException, Depends
from fastapi import WebSocket
from fastapi.websockets import WebSocketDisconnect

from backend.src.utils.logging_config import setup_logging
from backend.src.models.chat_request import ChatRequest
from backend.src.models.chat_response import ChatResponse
from backend.src.services.agent_service import AgentService
from backend.src.metrics import PrometheusMiddleware, metrics_endpoint

# Setup logging as early as possible
setup_logging()
logger = logging.getLogger(__name__)

from backend.src.services.database import create_tables, fetch_user_by_username, create_new_user, save_user_memory, get_user_memory
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Dict, Optional

# ... existing imports ...

app = FastAPI(
    title="RAG Agent with Gemini Backend API",
    description="API for interacting with the RAG Agent, grounded by Qdrant and powered by Gemini.",
    version="1.0.0"
)

# Initialize database on startup
@app.on_event("startup")
async def startup_event():
    logger.info("Initializing database...")
    await create_tables()
    logger.info("Database initialization complete.")


# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:3001"],  # Allow your Docusaurus frontend
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add Prometheus middleware
app.add_middleware(PrometheusMiddleware)

# --- Authentication Models ---
class UserBase(BaseModel):
    username: str
    email: str

class UserCreate(UserBase):
    password: str

class UserInDB(UserBase):
    id: int # Changed to int to match SERIAL PRIMARY KEY
    hashed_password: str

class Token(BaseModel):
    access_token: str
    token_type: str

class LoginRequest(BaseModel):
    username: str
    password: str

# --- Authentication Endpoints ---
@app.post("/signup", response_model=Token)
async def signup(user: UserCreate):
    # Check if username or email already exists in DB
    existing_user_by_username = await fetch_user_by_username(user.username)
    if existing_user_by_username:
        raise HTTPException(status_code=400, detail="Username already registered")
    
    # In a real app, you would hash the password properly
    hashed_password = user.password 

    try:
        user_id = await create_new_user(user.username, user.email, hashed_password)
        # For simplicity, returning the user_id as part of the access_token for mock purposes
        access_token = f"mock_token_user_{user_id}"
        logger.info(f"User signed up: {user.username} with ID: {user_id}")
        return {"access_token": access_token, "token_type": "bearer"}
    except Exception as e:
        logger.error(f"Signup error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error during signup")

@app.post("/login", response_model=Token)
async def login(request: LoginRequest):
    user_record = await fetch_user_by_username(request.username)
    
    if not user_record or user_record['hashed_password'] != request.password:
        raise HTTPException(status_code=400, detail="Incorrect username or password")
    
    user_id = user_record['id']
    access_token = f"mock_token_user_{user_id}"
    logger.info(f"User logged in: {request.username} with ID: {user_id}")
    return {"access_token": access_token, "token_type": "bearer"}

# --- Dependency to get current user from token ---
async def get_current_user_id(request: Request) -> Optional[int]:
    auth_header = request.headers.get("Authorization")
    if not auth_header:
        return None
    
    try:
        token_type, token = auth_header.split(" ")
        if token_type.lower() == "bearer" and token.startswith("mock_token_user_"):
            user_id = int(token.split("_")[-1])
            return user_id
    except ValueError:
        pass
    return None

def get_agent_service() -> AgentService:
    """Dependency injector for AgentService."""
    return AgentService()

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    agent_service: AgentService = Depends(get_agent_service),
    user_id: Optional[int] = Depends(get_current_user_id)
):
    """
    Endpoint to send a query to the RAG Agent and get a grounded response.
    """
    logger.info(f"Received /chat request with query: '{request.query}' and selected_text: '{request.selected_text}' for user_id: {user_id}")
    
    user_memory: Dict[str, str] = {}
    if user_id:
        user_memory = await get_user_memory(user_id)
        logger.debug(f"Retrieved memory for user {user_id}: {user_memory}")

        # Simple logic to detect and save personal info (can be enhanced)
        if "my name is" in request.query.lower():
            name_match = request.query.lower().split("my name is", 1)
            if len(name_match) > 1:
                name = name_match[1].strip().split(" ")[0] # Take first word after "my name is"
                if name:
                    await save_user_memory(user_id, "name", name)
                    user_memory["name"] = name # Update current session memory
                    logger.info(f"Saved user name: {name} for user {user_id}")
        
        if "i am a" in request.query.lower() and "student" in request.query.lower():
            if "educational_background" not in user_memory:
                await save_user_memory(user_id, "educational_background", "student")
                user_memory["educational_background"] = "student"
                logger.info(f"Saved educational background: student for user {user_id}")

    # Pass user_id and user_memory to the agent service
    answer = await agent_service.get_response(request.query, request.selected_text, user_id, user_memory)
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
            try:
                message_data = json.loads(data)
                query = message_data.get("query", "")
                selected_text = message_data.get("selected_text", "")
                access_token = message_data.get("access_token") # Get access token from frontend
                user_id_from_frontend = message_data.get("user_id") # Get user_id from frontend
            except json.JSONDecodeError:
                query = data
                selected_text = ""
                access_token = None
                user_id_from_frontend = None

            current_user_id: Optional[int] = None
            if access_token and user_id_from_frontend is not None: 
                try:
                    if access_token.startswith("mock_token_user_"):
                        extracted_id = int(access_token.split("_")[-1])
                        if extracted_id == int(user_id_from_frontend): # Validate user_id against token
                            current_user_id = extracted_id
                except ValueError:
                    logger.warning(f"Invalid user_id or access_token format in WebSocket message.")
            
            user_memory: Dict[str, str] = {}
            if current_user_id:
                user_memory = await get_user_memory(current_user_id)
                logger.debug(f"Retrieved memory for user {current_user_id} via WebSocket: {user_memory}")

                # Simple logic to detect and save personal info
                if "my name is" in query.lower():
                    name_match = query.lower().split("my name is", 1)
                    if len(name_match) > 1:
                        name = name_match[1].strip().split(" ")[0]
                        if name:
                            await save_user_memory(current_user_id, "name", name)
                            user_memory["name"] = name
                            logger.info(f"Saved user name: {name} for user {current_user_id} via WebSocket")
                
                if "i am a" in query.lower() and "student" in query.lower():
                    if "educational_background" not in user_memory:
                        await save_user_memory(current_user_id, "educational_background", "student")
                        user_memory["educational_background"] = "student"
                        logger.info(f"Saved educational background: student for user {current_user_id} via WebSocket")


            # Get response from agent service
            agent_service = AgentService()
            response = await agent_service.get_response(query, selected_text, current_user_id, user_memory)

            # Send response back to client
            response_data = {
                "response": response,
                "query": query
            }
            await websocket.send_text(json.dumps(response_data))
    except WebSocketDisconnect:
        logger.info("WebSocket connection closed")
    except Exception as e:
        logger.error(f"WebSocket error: {e}", exc_info=True)
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