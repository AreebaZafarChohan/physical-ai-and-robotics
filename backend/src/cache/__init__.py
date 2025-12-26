import json
import redis
from typing import Any, Optional
from datetime import timedelta
import os


class RedisCache:
    """
    Redis cache utility class for caching operations
    """
    
    def __init__(self):
        """
        Initialize Redis connection using environment variables
        """
        self.redis_client = redis.Redis(
            host=os.getenv("REDIS_HOST", "localhost"),
            port=int(os.getenv("REDIS_PORT", 6379)),
            password=os.getenv("REDIS_PASSWORD", None),
            db=int(os.getenv("REDIS_DB", 0)),
            decode_responses=True  # Decode responses to strings automatically
        )
    
    def get(self, key: str) -> Optional[str]:
        """
        Get a value from Redis cache
        """
        try:
            value = self.redis_client.get(key)
            return value
        except redis.RedisError:
            # Log error or handle appropriately
            return None
    
    def set(self, key: str, value: Any, expire: Optional[timedelta] = None) -> bool:
        """
        Set a value in Redis cache
        """
        try:
            # Convert non-string values to JSON string
            if not isinstance(value, str):
                value = json.dumps(value)
                
            if expire:
                # Set with expiration
                self.redis_client.setex(key, expire, value)
            else:
                # Set without expiration
                self.redis_client.set(key, value)
            
            return True
        except redis.RedisError:
            # Log error or handle appropriately
            return False
    
    def delete(self, key: str) -> bool:
        """
        Delete a key from Redis cache
        """
        try:
            result = self.redis_client.delete(key)
            return result > 0  # Return True if key was deleted
        except redis.RedisError:
            # Log error or handle appropriately
            return False
    
    def exists(self, key: str) -> bool:
        """
        Check if a key exists in Redis cache
        """
        try:
            return self.redis_client.exists(key) > 0
        except redis.RedisError:
            # Log error or handle appropriately
            return False
    
    def expire(self, key: str, expire: timedelta) -> bool:
        """
        Set expiration for a key in Redis cache
        """
        try:
            return self.redis_client.expire(key, int(expire.total_seconds()))
        except redis.RedisError:
            # Log error or handle appropriately
            return False


# Create a singleton instance of the Redis cache
cache = RedisCache()


# Specific caching functions for personalization
async def get_cached_personalized_content(user_profile_hash: str, content_path: str) -> Optional[str]:
    """
    Get cached personalized content
    """
    key = f"personalized_content:{user_profile_hash}:{content_path}"
    return cache.get(key)


async def cache_personalized_content(
    user_profile_hash: str,
    content_path: str,
    content: str,
    ttl: timedelta = timedelta(hours=24)
) -> bool:
    """
    Cache personalized content with TTL
    """
    key = f"personalized_content:{user_profile_hash}:{content_path}"
    return cache.set(key, content, expire=ttl)


async def invalidate_personalized_content_cache(user_profile_hash: str, content_path: str) -> bool:
    """
    Invalidate cached personalized content
    """
    key = f"personalized_content:{user_profile_hash}:{content_path}"
    return cache.delete(key)


async def invalidate_user_profile_cache(user_profile_hash: str) -> bool:
    """
    Invalidate all cached content for a user profile
    This is more complex as we would need to use Redis pattern matching
    """
    pattern = f"personalized_content:{user_profile_hash}:*"
    # Note: KEYS command is not recommended in production
    # In a real implementation, we'd want to maintain a set of keys for each user
    try:
        keys = cache.redis_client.keys(pattern)
        if keys:
            return cache.redis_client.delete(*keys) > 0
        return False
    except redis.RedisError:
        return False


# User profile caching functions
async def get_cached_user_profile(user_id: int) -> Optional[dict]:
    """
    Get cached user profile
    """
    key = f"user_profile:{user_id}"
    cached_data = cache.get(key)
    if cached_data:
        import json
        return json.loads(cached_data)
    return None


async def cache_user_profile(user_id: int, profile_data: dict, ttl: timedelta = timedelta(hours=1)) -> bool:
    """
    Cache user profile with TTL
    """
    key = f"user_profile:{user_id}"
    return cache.set(key, profile_data, expire=ttl)


async def invalidate_user_profile_cache_by_id(user_id: int) -> bool:
    """
    Invalidate cached user profile by user ID
    """
    key = f"user_profile:{user_id}"
    return cache.delete(key)