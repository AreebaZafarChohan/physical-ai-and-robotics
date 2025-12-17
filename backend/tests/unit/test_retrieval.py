import os
import pytest
from unittest.mock import patch, AsyncMock
from qdrant_client import QdrantClient
from qdrant_client.http.exceptions import UnexpectedResponse

from backend.src.retrieval.qdrant_client import get_qdrant_client
from backend.src.retrieval.embedder import generate_embedding, get_cohere_client
from backend.src.utils.config import get_settings # Import get_settings to ensure environment is set up

# Ensure settings are loaded for tests
@pytest.fixture(autouse=True)
def load_test_settings():
    with patch.dict(os.environ, {
        "GEMINI_API_KEY": "dummy_gemini_key",
        "GEMINI_API_BASE_URL": "http://dummy_gemini_url.com",
        "QDRANT_URL": "http://dummy_qdrant_url.com",
        "QDRANT_API_KEY": "dummy_qdrant_key",
        "QDRANT_COLLECTION_NAME": "dummy_collection",
        "COHERE_API_KEY": "dummy_cohere_key"
    }, clear=True):
        # Clear lru_cache for get_settings and get_qdrant_client before each test
        get_settings.cache_clear()
        get_qdrant_client.cache_clear()
        get_cohere_client.cache_clear()
        yield

# --- Tests for Qdrant Client ---

@patch('qdrant_client.QdrantClient.get_collections', return_value={})
def test_get_qdrant_client_success(mock_get_collections):
    """Tests successful Qdrant client initialization."""
    client = get_qdrant_client()
    assert isinstance(client, QdrantClient)
    mock_get_collections.assert_called_once()

@patch('qdrant_client.QdrantClient.get_collections', side_effect=Exception("Mock Qdrant Unauthorized"))
def test_get_qdrant_client_connection_error(mock_get_collections):
    """Tests Qdrant client initialization failure due to connection error."""
    with pytest.raises(ConnectionError, match="An unexpected error occurred with Qdrant connection"):
        get_qdrant_client()
    mock_get_collections.assert_called_once()

@patch('qdrant_client.QdrantClient.get_collections', side_effect=Exception("Generic error"))
def test_get_qdrant_client_generic_error(mock_get_collections):
    """Tests Qdrant client initialization failure due to generic error."""
    with pytest.raises(ConnectionError, match="An unexpected error occurred with Qdrant connection"):
        get_qdrant_client()
    mock_get_collections.assert_called_once()

# --- Tests for Embedder ---

@patch('backend.src.retrieval.embedder.AsyncClient', autospec=True)
def test_get_cohere_client_success(mock_async_client_class):
    """Tests successful Cohere client initialization."""
    get_cohere_client.cache_clear()
    client = get_cohere_client()
    assert client is not None
    mock_async_client_class.assert_called_once_with("dummy_cohere_key")

@patch('backend.src.retrieval.embedder.AsyncClient', side_effect=Exception("Cohere init error"))
def test_get_cohere_client_failure(mock_async_client_class):
    """Tests Cohere client initialization failure."""
    get_cohere_client.cache_clear()
    with pytest.raises(ConnectionError, match="Could not initialize Cohere client"):
        get_cohere_client()
    mock_async_client_class.assert_called_once_with("dummy_cohere_key")

@pytest.mark.asyncio
@patch('backend.src.retrieval.embedder.get_cohere_client')
async def test_generate_embedding_success(mock_get_cohere_client):
    """Tests successful embedding generation."""
    mock_cohere_client = AsyncMock()
    mock_get_cohere_client.return_value = mock_cohere_client
    mock_cohere_client.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]

    text = "test query"
    embedding = await generate_embedding(text)
    assert embedding == [0.1, 0.2, 0.3]
    mock_cohere_client.embed.assert_called_once_with(
        texts=[text], model="embed-english-v3.0", input_type="search_query"
    )

@pytest.mark.asyncio
@patch('backend.src.retrieval.embedder.get_cohere_client')
async def test_generate_embedding_empty_response(mock_get_cohere_client):
    """Tests embedding generation when Cohere returns empty embeddings."""
    mock_cohere_client = AsyncMock()
    mock_get_cohere_client.return_value = mock_cohere_client
    mock_cohere_client.embed.return_value.embeddings = []

    text = "test query"
    with pytest.raises(ValueError, match="Cohere embedding response was empty."):
        await generate_embedding(text)

@pytest.mark.asyncio
@patch('backend.src.retrieval.embedder.get_cohere_client', side_effect=ConnectionError("Client error"))
async def test_generate_embedding_client_error(mock_get_cohere_client):
    """Tests embedding generation when Cohere client errors."""
    text = "test query"
    with pytest.raises(RuntimeError, match="Could not generate embedding"):
        await generate_embedding(text)
