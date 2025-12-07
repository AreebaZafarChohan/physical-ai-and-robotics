import time # Added import
from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
from uuid import UUID

from backend.app.services.embedding_generator import generate_embeddings
from backend.app.vector_db import get_qdrant_client, QDRANT_COLLECTION_NAME
from qdrant_client import models
from backend.app.metrics import record_query_response_time # Added import

router = APIRouter()

class QueryRequest(BaseModel):
    query: str
    mode: str # Literal["book", "course"]
    user_id: Optional[UUID] = None

class Chunk(BaseModel):
    text: str
    metadata: Dict[str, Any]

class QueryResponse(BaseModel):
    relevant_chunks: List[Chunk]
    citation_links: List[str]

# Define a separate collection name for course materials
QDRANT_COURSE_COLLECTION_NAME = "course_materials"

@router.post("/query", response_model=QueryResponse)
async def query_content(request: QueryRequest):
    start_time = time.perf_counter() # Start timing
    try:
        if request.mode not in ["book", "course"]:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Invalid mode. Must be 'book' or 'course'.")

        query_embedding = generate_embeddings(texts=[request.query])
        if not query_embedding:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to generate query embedding."
            )
        
        qdrant_client = get_qdrant_client()
        search_results = []

        if request.mode == "book":
            book_search_result = qdrant_client.search(
                collection_name=QDRANT_COLLECTION_NAME, # book_chapters
                query_vector=query_embedding[0],
                limit=5,
                append_payload=True,
            )
            search_results.extend(book_search_result)
        elif request.mode == "course":
            book_search_result = qdrant_client.search(
                collection_name=QDRANT_COLLECTION_NAME, # book_chapters
                query_vector=query_embedding[0],
                limit=3, # Fewer results from book, more from course
                append_payload=True,
            )
            search_results.extend(book_search_result)

            course_search_result = qdrant_client.search(
                collection_name=QDRANT_COURSE_COLLECTION_NAME,
                query_vector=query_embedding[0],
                limit=7, # More results from course materials
                append_payload=True,
            )
            search_results.extend(course_search_result)
        
        relevant_chunks: List[Chunk] = []
        citation_links: List[str] = []

        for hit in search_results:
            chunk_text = hit.payload.get("text_content")
            source_link = hit.payload.get("source_link")
            
            if chunk_text:
                relevant_chunks.append(Chunk(text=chunk_text, metadata=hit.payload))
            if source_link and source_link not in citation_links:
                citation_links.append(source_link)
        
        response = QueryResponse(relevant_chunks=relevant_chunks, citation_links=citation_links)
        
        end_time = time.perf_counter() # End timing
        record_query_response_time(endpoint="/query", duration_ms=(end_time - start_time) * 1000, status_code=status.HTTP_200_OK)
        return response

    except HTTPException as e:
        end_time = time.perf_counter() # End timing for error
        record_query_response_time(endpoint="/query", duration_ms=(end_time - start_time) * 1000, status_code=e.status_code)
        raise e
    except Exception as e:
        end_time = time.perf_counter() # End timing for error
        record_query_response_time(endpoint="/query", duration_ms=(end_time - start_time) * 1000, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=f"Internal server error: {e}")