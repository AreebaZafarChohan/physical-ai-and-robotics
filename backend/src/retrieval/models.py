from pydantic import BaseModel
from typing import List, Dict, Any, Optional

class Metadata(BaseModel):
    source_url: str
    section: str
    heading: str
    chunk_index: int

class RetrievedChunk(BaseModel):
    text: str
    score: float
    metadata: Metadata

class Query(BaseModel):
    query: str
    top_k: int = 5

class RetrievalResult(BaseModel):
    query: str
    embedding_model: str
    top_k: int
    retrieved_chunks: List[RetrievedChunk]
