from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.security import APIKeyHeader
from backend.src.retrieval.models import Query, RetrievalResult
from backend.src.retrieval.main import RetrievalMain
from backend.src.retrieval.config import Config
import logging

logger = logging.getLogger(__name__)

app = FastAPI()
retrieval_main: RetrievalMain | None = None

def get_retrieval_main():
    global retrieval_main
    if retrieval_main is None:
        retrieval_main = RetrievalMain()
    return retrieval_main

api_key_header = APIKeyHeader(name="X-API-Key", auto_error=False)

async def verify_api_key(api_key: str = Depends(api_key_header)):
    if not api_key or api_key != Config.COHERE_API_KEY: # Using COHERE_API_KEY as a placeholder for a general API key
        logger.warning("Unauthorized access attempt due to invalid or missing API key.")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
        )
    return api_key

@app.post("/retrieve", response_model=RetrievalResult)
async def retrieve_chunks_endpoint(query: Query, api_key: str = Depends(verify_api_key)):
    """
    Exposes the RAG retrieval functionality via a FastAPI endpoint.
    Takes a query and returns relevant chunks.
    Requires API key authentication.
    """
    logger.info(f"API request received for query: '{query.query}'")
    try:
        rm = get_retrieval_main()
        logger.info(f"Successfully processed query: '{query.query}'")
        result = rm.retrieve_chunks(query)
        print(result)
        return result
    except Exception as e:
        logger.error(f"Error processing retrieval request for query '{query.query}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Internal Server Error: {str(e)}")



