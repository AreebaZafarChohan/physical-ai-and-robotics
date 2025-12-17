import logging
from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI
from agents.run import RunConfig
from backend.src.models.gemini_model import GeminiModel
from backend.src.agent.instructions import SYSTEM_PROMPT
from backend.src.agent.tools import retrieve_context
from dotenv import load_dotenv
logger = logging.getLogger(__name__)

load_dotenv()

class RAGAgent:
    def __init__(self):
        gemini_model_instance = GeminiModel()
        self.agent = Agent(
            name="RAG Chatbot Agent",
            instructions=SYSTEM_PROMPT,
            tools=[retrieve_context]
        )
        self.run_config = RunConfig(
            model=gemini_model_instance.chat_model,
            model_provider=gemini_model_instance.client,
        )
        logger.info("RAG Agent initialized with Gemini model and retrieve_context tool.")

    async def run(self, user_query: str) -> str:
        """
        Runs the RAG agent with a user query.
        """
        logger.info(f"RAG Agent received query: {user_query}")
        try:
            run_result = await Runner.run(self.agent, user_query, run_config=self.run_config)
            response = run_result.final_output
        except Exception as e:
            logger.error(f"An error occurred during agent execution: {e}", exc_info=True)
            raise RuntimeError(f"Failed to run agent: {e}")
        logger.info(f"RAG Agent response: {response}")
        return response

# Example usage
if __name__ == "__main__":
    import asyncio
    import os
    from backend.src.utils.logging_config import setup_logging
    from backend.src.utils.config import get_settings # To ensure env vars are loaded

    setup_logging()

    # Set dummy environment variables for testing purposes
    os.environ["GEMINI_API_KEY"] = "test_gemini_key"
    os.environ["GEMINI_API_BASE_URL"] = "http://localhost:8080/v1"
    os.environ["QDRANT_URL"] = "http://localhost:6333"  # Adjust if Qdrant is elsewhere
    os.environ["QDRANT_API_KEY"] = ""
    os.environ["QDRANT_COLLECTION_NAME"] = "test_collection" # Ensure this collection exists or is mocked for testing
    os.environ["COHERE_API_KEY"] = "dummy_cohere_key" # Replace with real if testing live

    # Ensure settings are loaded
    _ = get_settings()

    async def main_agent_test():
        try:
            rag_agent = RAGAgent()
            # This test won't actually retrieve context or call Gemini without proper setup
            # but it verifies the agent instantiation.
            print("RAGAgent instantiated successfully.")
            # Example run - this will likely fail if Qdrant/Gemini are not actually available
            # response = await rag_agent.run("What is the main topic of this document?")
            # print(f"Agent raw response: {response}")
        except Exception as e:
            print(f"Error during RAG Agent test: {e}")

    asyncio.run(main_agent_test())
