import logging
from qdrant_client import QdrantClient
from qdrant_client.http.exceptions import UnexpectedResponse
from functools import lru_cache
import os # Import os for example usage

from backend.src.utils.config import get_settings

logger = logging.getLogger(__name__)

@lru_cache()
def get_qdrant_client() -> QdrantClient:
    settings = get_settings()
    try:
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            prefer_grpc=True  # Use gRPC for better performance
        )
        # Test connection
        client.get_collections()
        logger.info(f"Successfully connected to Qdrant at {settings.QDRANT_URL}")
        return client
    except UnexpectedResponse as e:
        logger.error(f"Failed to connect to Qdrant or authenticate: {e}")
        raise ConnectionError(f"Could not connect to Qdrant: {e}")
    except Exception as e:
        logger.error(f"An unexpected error occurred while connecting to Qdrant: {e}")
        raise ConnectionError(f"An unexpected error occurred with Qdrant connection: {e}")


# Example usage
if __name__ == "__main__":
    # Ensure environment variables are set for testing
    os.environ["GEMINI_API_KEY"] = "test_gemini_key"
    os.environ["GEMINI_API_BASE_URL"] = "http://localhost:8080/v1"
    os.environ["QDRANT_URL"] = "http://localhost:6333" # Example for local Qdrant
    os.environ["QDRANT_API_KEY"] = "" # No API key for local example
    os.environ["QDRANT_COLLECTION_NAME"] = "test_collection"
    os.environ["COHERE_API_KEY"] = "test_cohere_key"

    from backend.src.utils.logging_config import setup_logging
    setup_logging()

    try:
        qdrant_client = get_qdrant_client()
        print(f"Qdrant client object: {qdrant_client}")
        # Further operations with qdrant_client can be added here
    except ConnectionError as e:
        print(f"Error: {e}")