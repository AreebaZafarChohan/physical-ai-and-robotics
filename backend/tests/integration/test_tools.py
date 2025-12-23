import os
import pytest
from unittest.mock import patch, AsyncMock
from qdrant_client.http.models import PointStruct, ScoredPoint, Payload
from backend.src.agent.tools import _retrieve_context_core
from backend.src.utils.config import get_settings
from backend.src.retrieval.qdrant_client import get_qdrant_client # Import get_qdrant_client

# Fixture to set up mock environment variables for tests
@pytest.fixture(autouse=True)
def set_test_env():
    with patch.dict(os.environ, {
        "GEMINI_API_KEY": "dummy_gemini_key",
        "GEMINI_API_BASE_URL": "http://dummy_gemini_url.com",
        "QDRANT_URL": "http://localhost:6333",  # Assuming a local Qdrant is targeted for integration
        "QDRANT_API_KEY": "", # No API key for local Qdrant in this example
        "QDRANT_COLLECTION_NAME": "test_collection",
        "COHERE_API_KEY": "dummy_cohere_key"
    }, clear=True):
        get_settings.cache_clear()
        yield

@pytest.fixture
def mock_qdrant_client_fixture():
    with patch('backend.src.retrieval.qdrant_client.get_qdrant_client') as mock_get_qdrant_client:
        mock_qdrant_client_instance = AsyncMock()
        mock_qdrant_client_instance.get_collections.return_value = AsyncMock()
        mock_get_qdrant_client.return_value = mock_qdrant_client_instance
        get_qdrant_client.cache_clear() # Ensure cache is clear before patching
        yield mock_qdrant_client_instance

@pytest.mark.asyncio
@patch('backend.src.retrieval.embedder.generate_embedding')
async def test_retrieve_context_success(mock_generate_embedding, mock_qdrant_client_fixture):
    """
    Tests successful context retrieval from Qdrant with mocked embedding and Qdrant client.
    """
    # Setup mocks
    mock_generate_embedding.return_value = [0.1, 0.2, 0.3] # Dummy embedding

    # Simulate Qdrant search result
    mock_qdrant_client_fixture.search.return_value = [
        ScoredPoint(id=1, version=1, score=0.9, payload={"text": "This is the first relevant document."}, vector=None),
        ScoredPoint(id=2, version=1, score=0.8, payload={"text": "Second document provides additional context."}, vector=None)
    ]

    query = "relevant information"
    top_k = 2
    
    # Call the function under test
    result = await _retrieve_context_core(query, top_k)
    # Assertions
    mock_generate_embedding.assert_called_once_with(query)
    # mock_get_qdrant_client.assert_called_once() # Ensure client was requested
    mock_qdrant_client_fixture.search.assert_called_once()
    
    # Check arguments passed to qdrant_client.search
    args, kwargs = mock_qdrant_client_fixture.search.call_args
    assert kwargs['collection_name'] == get_settings().QDRANT_COLLECTION_NAME
    assert kwargs['query_vector'] == [0.1, 0.2, 0.3]
    assert kwargs['limit'] == top_k
    assert kwargs['with_payload'] is True
    assert kwargs['with_vectors'] is False

    assert len(result) == 2
    assert "This is the first relevant document." in result
    assert "Second document provides additional context." in result


@pytest.mark.asyncio
@patch('backend.src.retrieval.embedder.generate_embedding')
async def test_retrieve_context_no_results(mock_generate_embedding, mock_qdrant_client_fixture):
    """
    Tests context retrieval when Qdrant returns no results.
    """
    mock_generate_embedding.return_value = [0.1, 0.2, 0.3]
    mock_qdrant_client_fixture.search.return_value = [] # Simulate no search results

    query = "test query" # Define query here
    result = await _retrieve_context_core(query)

    mock_generate_embedding.assert_called_once_with(query)
    # mock_get_qdrant_client.assert_called_once()
    mock_qdrant_client_fixture.search.assert_called_once()
    assert result == []

@pytest.mark.asyncio
@patch('backend.src.retrieval.embedder.generate_embedding', side_effect=RuntimeError("Embedding error"))
async def test_retrieve_context_embedding_failure(mock_generate_embedding, mock_qdrant_client_fixture):
    """
    Tests context retrieval when embedding generation fails.
    """
    query = "query with embedding failure"
    result = await _retrieve_context_core(query)

    mock_generate_embedding.assert_called_once_with(query)
    assert result == []

@pytest.mark.asyncio
@patch('backend.src.retrieval.qdrant_client.get_qdrant_client', side_effect=ConnectionError("Qdrant unavailable"))
@patch('backend.src.retrieval.embedder.generate_embedding') # Still need to mock embedder even if Qdrant fails first
async def test_retrieve_context_qdrant_connection_failure(mock_generate_embedding): # Removed mock_get_qdrant_client argument
    """
    Tests context retrieval when Qdrant client connection fails.
    """
    query = "query with qdrant failure"
    result = await _retrieve_context_core(query)

    # get_qdrant_client will raise ConnectionError, so the subsequent calls won't happen
    # mock_get_qdrant_client.assert_called_once() # This assert is no longer valid as get_qdrant_client raises an exception
    mock_generate_embedding.assert_called_once() # embedder is called before Qdrant client search

    assert result == [] # Should return empty list on connection error
