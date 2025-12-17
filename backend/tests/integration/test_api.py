import os
import pytest
from unittest.mock import patch, AsyncMock
from fastapi.testclient import TestClient
from backend.src.services.agent_service import AgentService
from backend.src.api.main import app, get_agent_service
from backend.src.agent.instructions import FALLBACK_RESPONSE
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
        yield

@pytest.mark.asyncio
async def test_chat_endpoint_success():
    """
    Tests the /chat endpoint for a successful, grounded response.
    """
    mock_agent_service = AsyncMock(spec=AgentService)
    mock_agent_service.chat_with_agent.return_value = "This is a grounded answer about the query."

    app.dependency_overrides[get_agent_service] = lambda: mock_agent_service
    try:
        client = TestClient(app)
        response = client.post("/chat", json={"query": "What is x?"})

        assert response.status_code == 200
        assert response.json() == {"response": "This is a grounded answer about the query."}
        mock_agent_service.chat_with_agent.assert_called_once_with("What is x?")
    finally:
        del app.dependency_overrides[get_agent_service]

@pytest.mark.asyncio
async def test_chat_endpoint_no_context_fallback():
    """
    Tests the /chat endpoint when the agent returns the fallback response.
    """
    mock_agent_service = AsyncMock(spec=AgentService)
    mock_agent_service.chat_with_agent.return_value = FALLBACK_RESPONSE

    app.dependency_overrides[get_agent_service] = lambda: mock_agent_service
    try:
        client = TestClient(app)
        response = client.post("/chat", json={"query": "Tell me about purple elephants."})

        assert response.status_code == 200
        assert response.json() == {"response": FALLBACK_RESPONSE}
        mock_agent_service.chat_with_agent.assert_called_once_with("Tell me about purple elephants.")
    finally:
        del app.dependency_overrides[get_agent_service]

@pytest.mark.asyncio
async def test_chat_endpoint_internal_error():
    """
    Tests the /chat endpoint when an internal error occurs in AgentService.
    """
    mock_agent_service = AsyncMock(spec=AgentService)
    mock_agent_service.chat_with_agent.side_effect = Exception("Simulated internal error")

    app.dependency_overrides[get_agent_service] = lambda: mock_agent_service
    try:
        client = TestClient(app)
        response = client.post("/chat", json={"query": "Query causing error"})

        assert response.status_code == 500
        assert "detail" in response.json()
        assert "unexpected error occurred" in response.json()["detail"].lower()
        mock_agent_service.chat_with_agent.assert_called_once_with("Query causing error")
    finally:
        del app.dependency_overrides[get_agent_service]

@pytest.mark.asyncio
async def test_chat_endpoint_invalid_request_body():
    """
    Tests the /chat endpoint with an invalid request body.
    """
    client = TestClient(app)
    response = client.post("/chat", json={"invalid_field": "value"})

    assert response.status_code == 422 # Unprocessable Entity for Pydantic validation errors
    assert "detail" in response.json()
    assert "query" in response.json()["detail"][0]["loc"]
    assert "field required" in response.json()["detail"][0]["msg"].lower()
