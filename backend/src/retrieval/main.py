import logging
from typing import List
from backend.src.retrieval.models import Query, RetrievedChunk, RetrievalResult, Metadata
from backend.src.retrieval.services import CohereService, QdrantService
from backend.src.retrieval.config import Config

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RetrievalMain:
    def __init__(self):
        self.cohere_service = CohereService()
        self.qdrant_service = QdrantService()
        self.vector_size = 1024 # Cohere embed-english-v3.0 typically produces 1024-dim vectors

        logger.info("Initializing RetrievalMain: Ensuring Qdrant collection exists.")
        try:
            self.qdrant_service.create_collection(vector_size=self.vector_size)
            logger.info(f"Qdrant collection '{self.qdrant_service.collection_name}' ensured.")
        except Exception as e:
            logger.error(f"Failed to ensure Qdrant collection: {e}")
            raise

    def retrieve_chunks(self, query: Query) -> RetrievalResult:
        logger.info(f"Retrieving chunks for query: '{query.query}' with top_k: {query.top_k}")
        try:
            query_embedding = self.cohere_service.embed_query(query.query)
            logger.debug("Query embedded successfully.")
        except Exception as e:
            logger.error(f"Error embedding query '{query.query}': {e}")
            raise

        try:
            # Search Qdrant for similar vectors
            search_results_obj = self.qdrant_service.search(
                query_vector=query_embedding,
                limit=query.top_k
            )
            # Extract the actual list of scored points from the QueryResponse object
            search_results = search_results_obj.points
            logger.info(f"Retrieved {len(search_results)} chunks from Qdrant.")
        except Exception as e:
            logger.error(f"Error searching Qdrant for query '{query.query}': {e}")
            raise

        retrieved_chunks: List[RetrievedChunk] = []
        for result in search_results:
            try:
                # Map Qdrant payload fields to our Metadata model
                metadata = Metadata(
                    source_url=result.payload.get("url", ""), # Map 'url' to 'source_url'
                    section=result.payload.get("section", ""), # Provide default if not present
                    heading=result.payload.get("heading", ""),   # Provide default if not present
                    chunk_index=result.payload.get("chunk_id", 0) # Map 'chunk_id' to 'chunk_index'
                )
                retrieved_chunks.append(
                    RetrievedChunk(
                        text=result.payload.get("text", ""), # Assuming 'text' is stored in payload
                        score=result.score,
                        metadata=metadata
                    )
                )
                logger.debug(f"Processed chunk with ID: {result.id}, score: {result.score}")
            except Exception as e:
                logger.warning(f"Error processing Qdrant search result (ID: {result.id}): {e}")
                # Depending on strictness, might want to skip or raise
        
        final_result = RetrievalResult(
            query=query.query,
            embedding_model="embed-english-v3.0",
            top_k=query.top_k,
            retrieved_chunks=retrieved_chunks
        )
        logger.info(f"Retrieval complete for query '{query.query}'. Total chunks returned: {len(retrieved_chunks)}")
        return final_result

