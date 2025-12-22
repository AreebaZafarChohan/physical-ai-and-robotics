# backend/tests/conftest.py
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy import select
from sqlalchemy.pool import StaticPool
from backend.src.database import Base, get_db
from backend.src.models.user import User

pytest_plugins = ["pytest_asyncio"] # Add this line

# Import middleware to match main app's configuration
from fastapi.middleware.cors import CORSMiddleware
from backend.src.metrics import PrometheusMiddleware # Assuming this is the correct import path

# --- Test Database Setup (Asynchronous SQLite) ---
SQLALCHEMY_DATABASE_URL = "sqlite+aiosqlite:///:memory:"
async_test_engine = create_async_engine( # Use async_test_engine
    SQLALCHEMY_DATABASE_URL,
    connect_args={"check_same_thread": False},
    poolclass=StaticPool,
)
TestingAsyncSessionLocal = async_sessionmaker(async_test_engine, class_=AsyncSession, expire_on_commit=False)

@pytest.fixture(name="session")
async def session_fixture():
    async with async_test_engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all) # Create tables
    
    async with TestingAsyncSessionLocal() as session:
        yield session
    
    async with async_test_engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all) # Drop tables

# --- Fixture for a test client that uses a dedicated test app ---
@pytest.fixture(name="client")
async def client_fixture(session: AsyncSession): # session is now AsyncSession
    from backend.src.api.auth import router as user_router
    from backend.src.api.onboarding import router as onboarding_router
    
    test_app = FastAPI() 

    # Add middleware to match the main app (Crucial for consistent behavior)
    test_app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://localhost:3000", "http://localhost:3001"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    test_app.add_middleware(PrometheusMiddleware) # Add Prometheus middleware
    
    # Override the get_db dependency for this test app
    async def override_get_db(): # Make this async to yield AsyncSession
        yield session

    test_app.dependency_overrides[get_db] = override_get_db

    test_app.include_router(user_router, prefix="/api/v1/users", tags=["User Management"])
    test_app.include_router(onboarding_router, prefix="/api/v1/onboarding", tags=["Onboarding"])

    # Change async with to with TestClient as it's a synchronous context manager
    with TestClient(test_app) as client: # CORRECTED
        yield client
    test_app.dependency_overrides.clear() # Clear overrides after test