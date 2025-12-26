import pytest
from unittest.mock import patch, MagicMock
from backend.src.cache import (
    RedisCache,
    get_cached_personalized_content,
    cache_personalized_content,
    invalidate_personalized_content_cache,
    get_cached_user_profile,
    cache_user_profile,
    invalidate_user_profile_cache_by_id
)


@pytest.fixture
def mock_redis_client():
    """Mock Redis client"""
    with patch('backend.src.cache.redis.Redis') as mock:
        yield mock.return_value


def test_redis_cache_get_set(mock_redis_client):
    """Test basic get and set operations in Redis cache"""
    mock_redis_client.get.return_value = "test_value"
    
    cache = RedisCache()
    cache.redis_client = mock_redis_client
    
    # Test get
    result = cache.get("test_key")
    assert result == "test_value"
    mock_redis_client.get.assert_called_once_with("test_key")
    
    # Reset the mock
    mock_redis_client.reset_mock()
    mock_redis_client.set.return_value = True
    
    # Test set
    result = cache.set("test_key", "test_value")
    assert result is True
    mock_redis_client.set.assert_called_once_with("test_key", "test_value")
    
    # Test set with expiration
    mock_redis_client.reset_mock()
    mock_redis_client.setex.return_value = True
    from datetime import timedelta
    result = cache.set("test_key", "test_value", expire=timedelta(seconds=60))
    assert result is True
    mock_redis_client.setex.assert_called_once()


@pytest.mark.asyncio
async def test_cache_personalized_content_functions():
    """Test the personalized content cache helper functions"""
    with patch('backend.src.cache.cache') as mock_cache:
        mock_cache.get.return_value = "cached_content"
        result = await get_cached_personalized_content("user_hash", "path")
        assert result == "cached_content"
        
        mock_cache.set.return_value = True
        result = await cache_personalized_content("user_hash", "path", "content")
        assert result is True
        
        mock_cache.delete.return_value = True
        result = await invalidate_personalized_content_cache("user_hash", "path")
        assert result is True


@pytest.mark.asyncio
async def test_cache_user_profile_functions():
    """Test the user profile cache helper functions"""
    with patch('backend.src.cache.cache') as mock_cache:
        # Mock the JSON parsing for get_cached_user_profile
        with patch('backend.src.cache.json.loads') as mock_json:
            mock_json.return_value = {"user_id": 123, "profile": "data"}
            mock_cache.get.return_value = '{"user_id": 123, "profile": "data"}'
            result = await get_cached_user_profile(123)
            assert result == {"user_id": 123, "profile": "data"}
            
            mock_cache.set.return_value = True
            result = await cache_user_profile(123, {"user_id": 123, "profile": "data"})
            assert result is True
            
            mock_cache.delete.return_value = True
            result = await invalidate_user_profile_cache_by_id(123)
            assert result is True