import logging
from functools import lru_cache
import cohere
from cohere import AsyncClient
try:
    from cohere.core.api_error import ApiError as CohereAPIError
except Exception:
    class CohereAPIError(Exception):
        """Fallback Cohere error for test environments"""
        pass
import os # Import os for example usage

from backend.src.utils.config import get_settings

logger = logging.getLogger(__name__)

@lru_cache()
def get_cohere_client() -> AsyncClient:
    settings = get_settings()
    try:
        client = AsyncClient(settings.COHERE_API_KEY)
        # Test client by listing models or a simple call if available and free
        # For simplicity, we'll assume successful initialization for now.
        logger.info("Successfully initialized Cohere client.")
        return client
    except Exception as e: # Catch any exception during client initialization
        logger.error(f"Failed to initialize Cohere client: {e}")
        raise ConnectionError(f"Could not initialize Cohere client: {e}")

async def generate_embedding(text: str) -> list[float]:
    """
    Generates an embedding for the given text using Cohere.
    """
    try:
        cohere_client = get_cohere_client()
    except ConnectionError as e:
        logger.error(f"Failed to get Cohere client: {e}")
        raise RuntimeError("Could not generate embedding due to client initialization failure.")
    try:
        # Use 'embed-english-v3.0' or another suitable model
        response = await cohere_client.embed(
            texts=[text], 
            model="embed-english-v3.0", 
            input_type="search_query"
        )
        if response.embeddings:
            return response.embeddings[0]
        else:
            # This is a specific condition where Cohere returns empty embeddings,
            # which should be a ValueError, not a generic RuntimeError.
            raise ValueError("Cohere embedding response was empty.")
    except ValueError:
        raise # Re-raise ValueError to be caught by the test
    except (CohereAPIError, Exception) as e: # Catch Cohere specific errors
        logger.error(f"Cohere API error during embedding for text: '{text[:50]}...' - {e}")
        raise RuntimeError(f"Could not generate embedding: {e}") # Re-raise as RuntimeError
    except Exception as e: # Catch any other unexpected errors during embedding generation
        logger.error(f"An unexpected error occurred during embedding generation for text: '{text[:50]}...' - {e}")
        raise RuntimeError(f"Could not generate embedding due to unexpected error: {e}")

# Example usage
if __name__ == "__main__":
    # Ensure environment variables are set for testing
    os.environ["GEMINI_API_KEY"] = "test_gemini_key"
    os.environ["GEMINI_API_BASE_URL"] = "http://localhost:8080/v1"
    os.environ["QDRANT_URL"] = "http://localhost:6333"
    os.environ["QDRANT_API_KEY"] = ""
    os.environ["QDRANT_COLLECTION_NAME"] = "test_collection"
    os.environ["COHERE_API_KEY"] = "dummy_cohere_key" # Replace with a real key for actual testing

    from backend.src.utils.logging_config import setup_logging
    setup_logging()

    async def main_embedder_test():
        test_text = "What is the capital of France?"
        try:
            embedding = await generate_embedding(test_text)
            print(f"Embedding generated (first 5 values): {embedding[:5]}...")
            print(f"Embedding length: {len(embedding)}")
        except (ConnectionError, RuntimeError, ValueError) as e:
            print(f"Error in embedding generation: {e}")

    import asyncio
    asyncio.run(main_embedder_test())
