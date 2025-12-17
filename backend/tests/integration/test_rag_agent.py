import os
import pytest
from unittest.mock import patch, AsyncMock, MagicMock

from agents import Agent, Runner # Import Runner
from agents.run import RunResult # Import RunResult to mock its return

from backend.src.agent.rag_agent import RAGAgent
from backend.src.agent.instructions import SYSTEM_PROMPT, FALLBACK_RESPONSE
from backend.src.agent.tools import retrieve_context
from backend.src.models.gemini_model import GeminiModel # For mocking internal components
from backend.src.utils.config import get_settings # To ensure environment is set up

# Fixture to set up mock environment variables for tests
@pytest.fixture(autouse=True)
def set_test_env():
    with patch.dict(os.environ, {
        "GEMINI_API_KEY": "dummy_gemini_key",
        "GEMINI_API_BASE_URL": "http://dummy_gemini_url.com",
        "QDRANT_URL": "http://localhost:6333",
        "QDRANT_API_KEY": "",
        "QDRANT_COLLECTION_NAME": "test_collection",
        "COHERE_API_KEY": "dummy_cohere_key"
    }, clear=True):
        get_settings.cache_clear()
        # Clear lru_cache for get_cohere_client (if used in setup of GeminiModel)
        # get_cohere_client.cache_clear()
        yield

@pytest.mark.asyncio
@patch('agents.Runner.run', autospec=True)
async def test_rag_agent_grounded_response(mock_runner_run):
    """
    Tests that the RAG agent provides a grounded response when relevant context is retrieved.
    """
    # Mock the Runner.run to return a specific grounded response
    mock_runner_run.return_value = RunResult(
        input="",
        new_items=[],
        raw_responses=[],
        input_guardrail_results=[],
        output_guardrail_results=[],
        tool_input_guardrail_results=[],
        tool_output_guardrail_results=[],
        context_wrapper=None,
        _last_agent=MagicMock(),
        final_output="Based on Document A, the capital of France is Paris."
    )
    agent = RAGAgent()
    user_query = "What is the capital of France?"
    response = await agent.run(user_query)

    # Assertions
    mock_runner_run.assert_called_once() # Ensure Runner.run was called
    # You can add more detailed assertions about the call arguments if needed,
    # e.g., checking the agent instance or the run_config if they were mocked.
    assert "Paris" in response
    assert "Based on Document A" in response # Check for grounding evidence

@pytest.mark.asyncio
@patch('agents.Runner.run')
async def test_rag_agent_ungrounded_response_no_retrieval(mock_runner_run):
    """
    Tests that the RAG agent responds with fallback when no relevant context is retrieved.
    """
    # Mock the Runner.run to return the fallback response
    mock_runner_run.return_value = AsyncMock(spec=RunResult)
    mock_runner_run.return_value.final_output = FALLBACK_RESPONSE

    agent = RAGAgent()
    user_query = "Tell me about purple elephants."
    response = await agent.run(user_query)

    # Assertions
    mock_runner_run.assert_called_once()
    assert response == FALLBACK_RESPONSE

@pytest.mark.asyncio
@patch('agents.Runner.run')
async def test_rag_agent_tool_failure_handling(mock_runner_run):
    """
    Tests that the RAG agent handles failures during tool execution gracefully.
    """
    # Simulate an error within Runner.run
    mock_runner_run.side_effect = Exception("Simulated tool execution error")

    agent = RAGAgent()
    user_query = "Query that causes tool failure."
    with pytest.raises(RuntimeError) as excinfo:
        await agent.run(user_query)

    mock_runner_run.assert_called_once()
    assert "Failed to run agent: Simulated tool execution error" in str(excinfo.value)