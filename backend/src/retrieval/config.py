import os
from dotenv import load_dotenv

load_dotenv()

class Config:
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "rag_validation")

print(f"DEBUG: QDRANT_URL: {Config.QDRANT_URL}")
print(f"DEBUG: QDRANT_API_KEY: {'*****' if Config.QDRANT_API_KEY else 'Not Set'}") # Mask API key for security
print(f"DEBUG: QDRANT_COLLECTION_NAME: {Config.QDRANT_COLLECTION_NAME}")

