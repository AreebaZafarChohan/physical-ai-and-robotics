import logging
from typing import List

from agents import function_tool
from qdrant_client.http.models import Filter

from backend.src.retrieval.qdrant_client import get_qdrant_client
from backend.src.retrieval.embedder import generate_embedding
from backend.src.utils.config import get_settings

logger = logging.getLogger(__name__)

# Assuming a simple structure for retrieved content
RetrievedContent = str

async def _retrieve_context_core(query: str, top_k: int = 5) -> List[RetrievedContent]:
    """
    Core logic to retrieve relevant context from the Qdrant vector database.
    """
    logger.info(f"Retrieving context for query: '{query}' with top_k: {top_k}")
    settings = get_settings()
    qdrant_client = get_qdrant_client()

    try:
        query_embedding = await generate_embedding(query)

        search_result = qdrant_client.query_points(
            collection_name=settings.QDRANT_COLLECTION_NAME,
            query=query_embedding, # Pass the generated embedding
            limit=top_k,
            # Ensure only payload is returned for text extraction
            with_payload=True,
            with_vectors=False
        )
        
        retrieved_texts = []
        for hit in search_result.points:
            if hit.payload and "text" in hit.payload: # Assuming payload has a 'text' key
                retrieved_texts.append(hit.payload["text"])
            else:
                logger.warning(f"Qdrant search hit with missing or non-text payload: {hit}")
        
        if not retrieved_texts:
            logger.info("No relevant context found in Qdrant.")
        
        return retrieved_texts

    except ConnectionError as e:
        logger.error(f"Qdrant or Cohere connection error during retrieval: {e}")
        return []
    except RuntimeError as e:
        logger.error(f"Embedding generation error during retrieval: {e}")
        return []
    except Exception as e:
        logger.error(f"An unexpected error occurred during context retrieval: {e}")
        return []

@function_tool
async def retrieve_context(query: str, top_k: int = 5) -> List[RetrievedContent]:
    """
    Retrieves relevant context from the Qdrant vector database based on a natural language query.

    Args:
        query (str): The natural language query to search for.
        top_k (int): The number of top relevant context chunks to retrieve. Defaults to 5.

    Returns:
        List[RetrievedContent]: A list of text payloads from the retrieved Qdrant points.
                                Returns an empty list if no relevant context is found or an error occurs.
    """
    return await _retrieve_context_core(query, top_k)


# Example usage for testing the tool function directly
if __name__ == "__main__":
    import asyncio
    import os
    from backend.src.utils.logging_config import setup_logging

    setup_logging()

    # Set dummy environment variables for testing purposes
    os.environ["GEMINI_API_KEY"] = "test_gemini_key"
    os.environ["GEMINI_API_BASE_URL"] = "http://localhost:8080/v1"
    os.environ["QDRANT_URL"] = "http://localhost:6333"  # Adjust if Qdrant is elsewhere
    os.environ["QDRANT_API_KEY"] = "" # Only if your Qdrant requires an API key
    os.environ["QDRANT_COLLECTION_NAME"] = "test_collection" # Ensure this collection exists or is mocked for testing
    os.environ["COHERE_API_KEY"] = "dummy_cohere_key" # Replace with real if testing live

    async def main_tool_test():
        query = "What is the capital of France?"
        try:
            results = await retrieve_context(query) # Call the decorated tool
            print("\n--- Retrieved Context ---")
            if results:
                for i, text in enumerate(results):
                    print(f"Result {i+1}: {text[:200]}...")
            else:
                print("No context retrieved.")
        except Exception as e:
            print(f"Error during tool test: {e}")

    asyncio.run(main_tool_test())