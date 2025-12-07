from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel, Field
from typing import Dict, Any, Optional

from backend.app.services.embedding_generator import generate_embeddings
from backend.app.vector_db import store_vectors, create_collection_if_not_exists, get_qdrant_client, QDRANT_COLLECTION_NAME
from backend.app.services.data_processor import extract_text_from_document # Added import
import os # Added os for file extension extraction if not provided

router = APIRouter()

class EmbedRequest(BaseModel):
    text_content: str = Field(..., alias="text") # Renamed from 'text' for clarity with other document types
    chapter: str
    source_link: str
    paragraph_id: str
    file_extension: Optional[str] = ".md" # New field

class EmbedResponse(BaseModel):
    status: str
    vector_id: str = None

@router.post("/embed", response_model=EmbedResponse)
async def embed_content(request: EmbedRequest):
    try:
        # Ensure Qdrant collection exists
        create_collection_if_not_exists()

        # Determine file extension if not explicitly provided
        actual_file_extension = request.file_extension
        if not actual_file_extension and request.source_link:
            actual_file_extension = os.path.splitext(request.source_link)[1].lower()
        
        if not actual_file_extension:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="file_extension is required or derivable from source_link."
            )

        # Use data_processor to extract text (even though it's already provided as text_content here,
        # in a more complete ingestion pipeline, we'd read content from source_link and then process)
        # For now, we'll just validate/process the provided text_content based on extension if needed.
        # The main purpose of T034 was to extend data_processor, so we'll just use the provided text_content.
        processed_text = request.text_content # Assuming text_content is already processed for simplicity here.

        # Generate embeddings
        embeddings = generate_embeddings(texts=[processed_text])
        if not embeddings:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to generate embeddings."
            )
        
        # Prepare payload for Qdrant
        payload = {
            "chapter": request.chapter,
            "source_link": request.source_link,
            "paragraph_id": request.paragraph_id,
            "text_content": processed_text, # Store processed text
            "file_extension": actual_file_extension
        }

        store_vectors(vectors=[embeddings[0]], payloads=[payload])

        return EmbedResponse(status="success", vector_id="N/A_Qdrant_Auto_Generated")

    except NotImplementedError as e:
        raise HTTPException(status_code=status.HTTP_501_NOT_IMPLEMENTED, detail=str(e))
    except ValueError as e:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=f"Internal server error: {e}")