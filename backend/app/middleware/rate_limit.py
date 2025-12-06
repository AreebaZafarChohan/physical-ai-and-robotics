import os
from fastapi import Request, Response, HTTPException
from fastapi_limiter import FastAPILimiter
from fastapi_limiter.depends import RateLimiter
import redis.asyncio as redis

# This should ideally be configured via dependency injection or a global setup in main.py
# For now, we'll initialize it here.
REDIS_URL = os.getenv("REDIS_URL", "redis://localhost:6379/0")

async def init_rate_limiter():
    redis_instance = redis.from_url(REDIS_URL, encoding="utf-8", decode_responses=True)
    await FastAPILimiter.init(redis_instance)

# This dependency can be used in FastAPI routes
# Example: @app.get("/items/", dependencies=[Depends(RateLimiter(times=2, seconds=5))])
