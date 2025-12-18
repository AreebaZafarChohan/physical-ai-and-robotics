
from cohere import Client
from typing import List, Dict, Any, Optional
import time
import logging

from qdrant_client import QdrantClient, models
from qdrant_client.http.models import Distance, VectorParams
from qdrant_client.http.exceptions import UnexpectedResponse

from backend.src.retrieval.config import Config

logger = logging.getLogger(__name__)

def exponential_backoff_retry(max_retries=5, initial_delay=1.0, backoff_factor=2):
    def decorator(func):
        def wrapper(*args, **kwargs):
            delay = initial_delay
            for i in range(max_retries):
                try:
                    return func(*args, **kwargs)
                except (Exception, UnexpectedResponse) as e: # Changed CohereError to Exception
                    logger.warning(f"Attempt {i+1}/{max_retries} failed for {func.__name__}: {e}")
                    if i < max_retries - 1:
                        time.sleep(delay)
                        delay *= backoff_factor
                    else:
                        logger.error(f"Max retries reached for {func.__name__}. Giving up.")
                        raise
        return wrapper
    return decorator

class CohereService:
    def __init__(self):
        self.client = Client(Config.COHERE_API_KEY)

    @exponential_backoff_retry()
    def embed_documents(self, texts: List[str]) -> List[List[float]]:
        response = self.client.embed(
            texts=texts,
            model="embed-english-v3.0", # As specified in plan.md
            input_type="search_document"
        )
        return response.embeddings

    @exponential_backoff_retry()
    def embed_query(self, text: str) -> List[float]:
        response = self.client.embed(
            texts=[text],
            model="embed-english-v3.0", # As specified in plan.md
            input_type="search_query"
        )
        return response.embeddings[0]

class QdrantService:
    def __init__(self):
        self.client = QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
        )
        self.collection_name = Config.QDRANT_COLLECTION_NAME

    @exponential_backoff_retry()
    def create_collection(self, vector_size: int, distance: Distance = Distance.COSINE):
        try:
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' exists. Skipping creation.")
        except Exception:
            logger.info(f"Collection not found. Creating '{self.collection_name}'.")
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=vector_size,
                    distance=distance
                )
            )



    @exponential_backoff_retry()
    def upsert_vectors(self, vectors: List[List[float]], payloads: List[Dict[str, Any]], ids: Optional[List[int]] = None):
        points = []
        for i, vector in enumerate(vectors):
            point = models.PointStruct(
                id=ids[i] if ids else None,
                vector=vector,
                payload=payloads[i]
            )
            points.append(point)

        self.client.upsert(
            collection_name=self.collection_name,
            wait=True,
            points=points
        )

    @exponential_backoff_retry()
    def search(self, query_vector: List[float], limit: int = 5) -> List[models.ScoredPoint]:
        print(f"DEBUG: Type of self.client in QdrantService.search: {type(self.client)}")
        print(f"DEBUG: Methods available on self.client: {sorted([attr for attr in dir(self.client) if not attr.startswith('__')])}")
        search_result = self.client.query_points( # Changed to .query_points
            collection_name=self.collection_name,
            query=query_vector, # Pass vector as 'query' keyword argument
            limit=limit,
        )
        return search_result

