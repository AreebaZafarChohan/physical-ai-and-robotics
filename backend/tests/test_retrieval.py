import pytest
from unittest.mock import MagicMock, patch
from backend.src.retrieval.models import Query, RetrievedChunk, Metadata, RetrievalResult
from backend.src.retrieval.main import RetrievalMain
from backend.src.retrieval.services import CohereService, QdrantService
from qdrant_client.http.models import ScoredPoint
from typing import List

@pytest.fixture
def mock_cohere_service():
    with patch('backend.src.retrieval.services.CohereService') as MockCohereServiceClass:
        # Create a mock instance of CohereService
        mock_instance = MockCohereServiceClass.return_value
        # Mock the embed_query method on the instance
        mock_instance.embed_query.return_value = [0.1] * 1024
        return mock_instance

@pytest.fixture
def mock_qdrant_service():
    with patch('backend.src.retrieval.services.QdrantService') as MockQdrantServiceClass:
        # Create a mock instance of QdrantService
        mock_instance = MockQdrantServiceClass.return_value
        # Mock create_collection to do nothing
        mock_instance.create_collection.return_value = None
        
        # Mock search results
        mock_scored_point1 = ScoredPoint(
            id=1,
            version=1,
            score=0.9,
            payload={
                "text": "This is a test chunk about ROS 2.",
                "source_url": "http://example.com/ros2",
                "section": "Intro",
                "heading": "What is ROS 2?",
                "chunk_index": 0
            },
            vector=None
        )
        mock_scored_point2 = ScoredPoint(
            id=2,
            version=1,
            score=0.85,
            payload={
                "text": "Another test chunk about robotics.",
                "source_url": "http://example.com/robotics",
                "section": "Concepts",
                "heading": "Robotics Overview",
                "chunk_index": 1
            },
            vector=None
        )
        mock_instance.search.return_value = [mock_scored_point1, mock_scored_point2]
        return mock_instance

@pytest.fixture
def retrieval_main(mock_cohere_service, mock_qdrant_service):
    # Patch the CohereService and QdrantService classes within the main module
    # so that RetrievalMain uses our mocked instances.
    with patch('backend.src.retrieval.main.CohereService', return_value=mock_cohere_service):
        with patch('backend.src.retrieval.main.QdrantService', return_value=mock_qdrant_service):
            # Patch Config values as well to avoid errors during initialization
            with patch('backend.src.retrieval.config.Config.COHERE_API_KEY', 'dummy_cohere_key'):
                with patch('backend.src.retrieval.config.Config.QDRANT_URL', 'http://mock-qdrant.com'):
                    with patch('backend.src.retrieval.config.Config.QDRANT_API_KEY', 'dummy_qdrant_key'):
                        yield RetrievalMain()



def test_retrieve_chunks_basic_accuracy(retrieval_main, mock_cohere_service, mock_qdrant_service):
    """
    Test basic retrieval accuracy: ensures the main logic calls services correctly
    and returns results in the expected format.
    """
    test_query = Query(query="What is ROS 2?", top_k=2)
    result = retrieval_main.retrieve_chunks(test_query)

    # Assert CohereService.embed_query was called
    mock_cohere_service.embed_query.assert_called_once_with(test_query.query)
    
    # Assert QdrantService.search was called with the correct embedding and limit
    mock_qdrant_service.search.assert_called_once()
    call_args, call_kwargs = mock_qdrant_service.search.call_args
    assert 'query_vector' in call_kwargs
    # Check that query_vector is the expected embedding from mock_cohere_service
    assert call_kwargs['query_vector'] == mock_cohere_service.embed_query.return_value
    assert call_kwargs['limit'] == test_query.top_k

    # Assert the overall structure and content of the result
    assert isinstance(result, RetrievalResult)
    assert result.query == test_query.query
    assert result.embedding_model == "embed-english-v3.0"
    assert result.top_k == test_query.top_k
    assert len(result.retrieved_chunks) == 2

    # Validate individual retrieved chunks
    assert all(isinstance(chunk, RetrievedChunk) for chunk in result.retrieved_chunks)
    assert result.retrieved_chunks[0].text == "This is a test chunk about ROS 2."
    assert result.retrieved_chunks[0].score == 0.9
    assert isinstance(result.retrieved_chunks[0].metadata, Metadata)
    assert result.retrieved_chunks[0].metadata.source_url == "http://example.com/ros2"

    assert result.retrieved_chunks[1].text == "Another test chunk about robotics."
    assert result.retrieved_chunks[1].score == 0.85
    assert isinstance(result.retrieved_chunks[1].metadata, Metadata)
    assert result.retrieved_chunks[1].metadata.source_url == "http://example.com/robotics"


def test_retrieve_chunks_metadata_integrity(retrieval_main, mock_cohere_service, mock_qdrant_service):
    """
    Test metadata integrity: ensures all expected metadata fields are present and have correct types.
    """
    test_query = Query(query="Metadata check", top_k=1)
    result = retrieval_main.retrieve_chunks(test_query)

    assert len(result.retrieved_chunks) > 0
    first_chunk = result.retrieved_chunks[0]
    metadata = first_chunk.metadata

    assert isinstance(metadata, Metadata)
    assert isinstance(metadata.source_url, str)
    assert isinstance(metadata.section, str)
    assert isinstance(metadata.heading, str)
    assert isinstance(metadata.chunk_index, int)
    
    # Optionally, check for non-empty values if appropriate for the data
    assert metadata.source_url != ""
    assert metadata.section != ""
    assert metadata.heading != ""
    # chunk_index can be 0, so no non-empty check needed

