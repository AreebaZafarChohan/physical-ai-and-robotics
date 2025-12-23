import logging
from typing import Optional, Dict

from backend.src.agent.rag_agent import RAGAgent
from backend.src.agent.instructions import FALLBACK_RESPONSE

logger = logging.getLogger(__name__)

class AgentService:
    def __init__(self):
        self.rag_agent = RAGAgent()
        logger.info("AgentService initialized with RAGAgent.")

    async def get_response(self, query: str, selected_text: Optional[str] = None, user_id: Optional[int] = None, user_memory: Optional[Dict[str, str]] = None) -> str:
        """
        Handles the conversation with the RAG agent, including tool invocation and response generation.
        Combines the query with selected text if provided.
        """
        # If selected text is provided, combine it with the query
        if selected_text and selected_text.strip():
            full_query = f"Context: {selected_text}\n\nQuestion: {query}"
            logger.info(f"AgentService received query with selected text: '{full_query}' for user_id: {user_id}")
        else:
            full_query = query
            logger.info(f"AgentService received query: '{query}' for user_id: {user_id}")

        try:
            # The agent's run method internally handles tool invocation and LLM reasoning
            # based on its system prompt and attached tools.
            agent_response = await self.rag_agent.run(full_query, user_id=user_id, user_memory=user_memory)

            # Post-processing or checking for fallback can happen here if agent doesn't do it explicitly
            if agent_response.strip() == FALLBACK_RESPONSE:
                logger.warning(f"Agent responded with fallback for query: '{full_query}' for user_id: {user_id}")

            return agent_response
        except Exception as e:
            logger.error(f"Error during agent chat for query '{full_query}' for user_id: {user_id}: {e}", exc_info=True)
            return "An error occurred while processing your request. Please try again later."

    async def chat_with_agent(self, user_query: str, user_id: Optional[int] = None, user_memory: Optional[Dict[str, str]] = None) -> str:
        """
        Handles the conversation with the RAG agent, including tool invocation and response generation.
        """
        logger.info(f"AgentService received chat query: '{user_query}' for user_id: {user_id}")
        return await self.get_response(user_query, user_id=user_id, user_memory=user_memory)

# Example usage
if __name__ == "__main__":
    import asyncio
    import os
    from backend.src.utils.logging_config import setup_logging
    from backend.src.utils.config import get_settings

    setup_logging()

    # Set dummy environment variables for testing purposes
    os.environ["GEMINI_API_KEY"] = "test_gemini_key"
    os.environ["GEMINI_API_BASE_URL"] = "http://localhost:8080/v1"
    os.environ["QDRANT_URL"] = "http://localhost:6333"
    os.environ["QDRANT_API_KEY"] = ""
    os.environ["QDRANT_COLLECTION_NAME"] = "test_collection"
    os.environ["COHERE_API_KEY"] = "dummy_cohere_key"

    _ = get_settings()

    async def main_agent_service_test():
        agent_service = AgentService()
        print("AgentService instantiated.")
        # This will not provide a meaningful answer without a running Qdrant and Gemini endpoint
        # However, it demonstrates the call flow.
        query_to_test = "What is the capital of France?"
        print(f"\nQuerying agent: '{query_to_test}'")
        response = await agent_service.chat_with_agent(query_to_test)
        print(f"Agent's response: {response}")

        query_unanswerable = "Tell me about purple elephants."
        print(f"\nQuerying agent: '{query_unanswerable}'")
        response_unanswerable = await agent_service.chat_with_agent(query_unanswerable)
        print(f"Agent's response: {response_unanswerable}")

    asyncio.run(main_agent_service_test())
