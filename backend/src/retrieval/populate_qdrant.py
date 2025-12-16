import os
from qdrant_client import QdrantClient, models
from cohere import Client
from dotenv import load_dotenv

# Load environment variables from .env
load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
COHERE_API_KEY = os.getenv("COHERE_API_KEY", "")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "rag_validation")

# Initialize Qdrant client
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Initialize Cohere client
cohere_client = Client(api_key=COHERE_API_KEY)

# Ensure collection exists (this will recreate if it exists for this example)
vector_size = 1024  # Cohere embed-english-v3.0 produces 1024-dim vectors
qdrant_client.recreate_collection(
    collection_name=COLLECTION_NAME,
    vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
)

print(f"Collection '{COLLECTION_NAME}' ensured.")

# Sample data
documents = [
    {
        "text": "ROS 2 is an open-source robotic middleware.",
        "source_url": "url1",
        "section": "intro",
        "heading": "ROS2",
        "chunk_index": 0,
    },
    {
        "text": "Gazebo is a powerful 3D robotics simulator.",
        "source_url": "url2",
        "section": "sim",
        "heading": "Gazebo",
        "chunk_index": 0,
    },
    {
        "text": "NVIDIA Isaac Sim provides a robotics simulation and synthetic data generation platform.",
        "source_url": "url3",
        "section": "isaac",
        "heading": "IsaacSim",
        "chunk_index": 0,
    },
    {
        "text": "Cognitive planning involves AI systems reasoning about actions.",
        "source_url": "url4",
        "section": "ai",
        "heading": "Cognition",
        "chunk_index": 0,
    },
]

texts_to_embed = [doc["text"] for doc in documents]
embeddings = cohere_client.embed(
    texts=texts_to_embed,
    model="embed-english-v3.0",
    input_type="search_document",
).embeddings

print("Documents embedded.")

points = []
for i, doc in enumerate(documents):
    # Create a payload for Qdrant, ensuring it matches the Metadata model
    payload = {
        "text": doc["text"],
        "source_url": doc["source_url"],
        "section": doc["section"],
        "heading": doc["heading"],
        "chunk_index": doc["chunk_index"],
    }
    points.append(
        models.PointStruct(
            id=i,  # Simple ID assignment
            vector=embeddings[i],
            payload=payload,
        )
    )

qdrant_client.upsert(
    collection_name=COLLECTION_NAME,
    wait=True,
    points=points,
)

print(f"Upserted {len(points)} points into collection '{COLLECTION_NAME}'.")