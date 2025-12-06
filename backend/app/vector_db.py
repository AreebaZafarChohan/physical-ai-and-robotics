import os
from typing import List, Dict, Any
from qdrant_client import QdrantClient, models
from uuid import uuid4 # Added import

QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_CLUSTER_URL = os.getenv("QDRANT_CLUSTER_URL")
QDRANT_COLLECTION_NAME = "book_chapters" # Defined as per task

if not QDRANT_API_KEY or not QDRANT_CLUSTER_URL:
    raise ValueError("QDRANT_API_KEY or QDRANT_CLUSTER_URL environment variables are not set.")

qdrant_client = QdrantClient(
    url=QDRANT_CLUSTER_URL,
    api_key=QDRANT_API_KEY,
)

def get_qdrant_client():
    return qdrant_client

def create_collection_if_not_exists():
    """
    Creates the Qdrant collection if it doesn't already exist.
    """
    try:
        qdrant_client.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE), # OpenAI embeddings usually 1536 dim
        )
        print(f"Collection '{QDRANT_COLLECTION_NAME}' created or recreated.")
    except Exception as e:
        print(f"Error creating collection: {e}")

def store_vectors(vectors: List[List[float]], payloads: List[Dict[str, Any]]):
    """
    Stores vectors and their associated payloads into the Qdrant collection.
    """
    if not vectors or not payloads:
        return

    points = []
    for i, vec in enumerate(vectors):
        points.append(
            models.PointStruct(
                id=str(uuid4()), # Generate a UUID for each point
                vector=vec,
                payload=payloads[i]
            )
        )
    
    try:
        qdrant_client.upsert(
            collection_name=QDRANT_COLLECTION_NAME,
            wait=True,
            points=points
        )
        print(f"Successfully stored {len(points)} vectors into '{QDRANT_COLLECTION_NAME}'.")
    except Exception as e:
        print(f"Error storing vectors: {e}")

