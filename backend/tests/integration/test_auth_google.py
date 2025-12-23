import os
import pytest
from unittest.mock import patch, AsyncMock
from fastapi.testclient import TestClient
from backend.src.api.main import app # Import app from main
from backend.src.models.user import User # Import User model
from backend.src.database import get_db, Base # Import get_db and Base to override it
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine, AsyncEngine
from sqlalchemy.orm import sessionmaker
from typing import Generator
import jwt # Import jwt here
import uuid # Import uuid for User creation
from backend.src.utils.hash import Hash # Import Hash for password hashing

# Mock database for testing
SQLALCHEMY_DATABASE_URL = "sqlite+aiosqlite:///./test.db" # Use aiosqlite driver
async_engine = create_async_engine(
    SQLALCHEMY_DATABASE_URL, connect_args={"check_same_thread": False}
)
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=async_engine, class_=AsyncSession)

# Setup test client
@pytest.fixture(name="client")
def client_fixture() -> Generator[TestClient, None, None]:
    with TestClient(app) as client:
        yield client

# Mock get_db dependency
@pytest.fixture(name="db_session")
async def db_session_fixture() -> Generator[AsyncSession, None, None]:
    async with async_engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all) # Create tables
    
    db = TestingSessionLocal()
    try:
        yield db
    finally:
        await db.close()
        async with async_engine.begin() as conn:
            await conn.run_sync(Base.metadata.drop_all) # Drop tables

@pytest.fixture(autouse=True)
def override_get_db(db_session: AsyncSession):
    app.dependency_overrides[get_db] = lambda: db_session
    yield
    del app.dependency_overrides[get_db]


@pytest.fixture(autouse=True)
def set_google_test_env():
    # Patch environment variables for Google OAuth
    with patch.dict(os.environ, {
        "GOOGLE_CLIENT_ID": "test_google_client_id",
        "GOOGLE_CLIENT_SECRET": "test_google_client_secret",
        "GOOGLE_REDIRECT_URI": "http://localhost:9000/api/v1/auth/google/callback",
        "FRONTEND_BASE_URL": "http://localhost:3000",
        "JWT_SECRET_KEY": "super-secret-jwt-key"
    }, clear=False): # Do not clear existing environment variables
        yield

@pytest.mark.asyncio
async def test_google_auth_redirect(client: TestClient):
    response = client.get("/api/v1/auth/google")
    assert response.status_code == 302
    assert "accounts.google.com/o/oauth2/v2/auth" in response.headers["location"]
    assert "client_id=test_google_client_id" in response.headers["location"]
    assert "redirect_uri=http%3A%2F%2Flocalhost%3A9000%2Fapi%2Fv1%2Fauth%2Fgoogle%2Fcallback" in response.headers["location"]
    assert "response_type=code" in response.headers["location"]
    assert "scope=openid+email+profile" in response.headers["location"]

@pytest.mark.asyncio
@patch("httpx.AsyncClient.post")
@patch("jwt.decode")
async def test_google_callback_new_user(mock_jwt_decode, mock_httpx_post, client: TestClient, db_session: AsyncSession):
    # Mock Google's token response
    mock_httpx_post.return_value = AsyncMock(
        json=lambda: {
            "id_token": "mock_id_token",
            "access_token": "mock_access_token"
        },
        raise_for_status=lambda: None
    )

    # Mock JWT decode of Google's ID token
    mock_jwt_decode.return_value = {
        "email": "newuser@example.com",
        "name": "New User",
        "sub": "google_new_user_id"
    }

    response = client.get("/api/v1/auth/google/callback?code=mock_code")
    assert response.status_code == 302
    assert "http://localhost:3000/auth/callback?token=" in response.headers["location"]

    # Verify user was created in the database
    user = await db_session.execute(select(User).filter_by(email="newuser@example.com"))
    user = user.scalars().first()
    assert user is not None
    assert user.google_id == "google_new_user_id"
    assert user.password is None # Password should be None for social login

    # Verify JWT token
    token = response.headers["location"].split("token=")[1]
    decoded_token = jwt.decode(token, os.getenv("JWT_SECRET_KEY"), algorithms=["HS256"])
    assert decoded_token["sub"] == "newuser@example.com"
    assert decoded_token["user_id"] == user.id

@pytest.mark.asyncio
@patch("httpx.AsyncClient.post")
@patch("jwt.decode")
async def test_google_callback_existing_user_no_google_id(mock_jwt_decode, mock_httpx_post, client: TestClient, db_session: AsyncSession):
    # Create an existing user without google_id
    existing_user = User(
        id=str(uuid.uuid4()),
        email="existing@example.com",
        password=Hash.bcrypt("password"),
        software_experience="beginner",
        hardware_experience="none"
    )
    db_session.add(existing_user)
    await db_session.commit()

    # Mock Google's token response
    mock_httpx_post.return_value = AsyncMock(
        json=lambda: {
            "id_token": "mock_id_token",
            "access_token": "mock_access_token"
        },
        raise_for_status=lambda: None
    )

    # Mock JWT decode of Google's ID token
    mock_jwt_decode.return_value = {
        "email": "existing@example.com",
        "name": "Existing User",
        "sub": "google_existing_user_id"
    }

    response = client.get("/api/v1/auth/google/callback?code=mock_code")
    assert response.status_code == 302
    assert "http://localhost:3000/auth/callback?token=" in response.headers["location"]

    # Verify user's google_id was updated
    user = await db_session.execute(select(User).filter_by(email="existing@example.com"))
    user = user.scalars().first()
    assert user is not None
    assert user.google_id == "google_existing_user_id"

    # Verify JWT token
    token = response.headers["location"].split("token=")[1]
    decoded_token = jwt.decode(token, os.getenv("JWT_SECRET_KEY"), algorithms=["HS256"])
    assert decoded_token["sub"] == "existing@example.com"
    assert decoded_token["user_id"] == user.id

@pytest.mark.asyncio
@patch("httpx.AsyncClient.post")
@patch("jwt.decode")
async def test_google_callback_existing_user_with_google_id(mock_jwt_decode, mock_httpx_post, client: TestClient, db_session: AsyncSession):
    # Create an existing user with google_id
    existing_user = User(
        id=str(uuid.uuid4()),
        email="existing_google@example.com",
        password=None,
        software_experience="expert",
        hardware_experience="intermediate",
        google_id="google_existing_user_id_2"
    )
    db_session.add(existing_user)
    await db_session.commit()

    # Mock Google's token response
    mock_httpx_post.return_value = AsyncMock(
        json=lambda: {
            "id_token": "mock_id_token",
            "access_token": "mock_access_token"
        },
        raise_for_status=lambda: None
    )

    # Mock JWT decode of Google's ID token
    mock_jwt_decode.return_value = {
        "email": "existing_google@example.com",
        "name": "Existing Google User",
        "sub": "google_existing_user_id_2"
    }

    response = client.get("/api/v1/auth/google/callback?code=mock_code")
    assert response.status_code == 302
    assert "http://localhost:3000/auth/callback?token=" in response.headers["location"]

    # Verify existing user was logged in
    user = await db_session.execute(select(User).filter_by(email="existing_google@example.com"))
    user = user.scalars().first()
    assert user is not None
    assert user.google_id == "google_existing_user_id_2"

    # Verify JWT token
    token = response.headers["location"].split("token=")[1]
    decoded_token = jwt.decode(token, os.getenv("JWT_SECRET_KEY"), algorithms=["HS256"])
    assert decoded_token["sub"] == "existing_google@example.com"
    assert decoded_token["user_id"] == user.id

@pytest.mark.asyncio
@patch("httpx.AsyncClient.post")
@patch("jwt.decode")
async def test_google_callback_missing_id_token(mock_jwt_decode, mock_httpx_post, client: TestClient):
    mock_httpx_post.return_value = AsyncMock(
        json=lambda: {}, # No id_token
        raise_for_status=lambda: None
    )
    response = client.get("/api/v1/auth/google/callback?code=mock_code")
    assert response.status_code == 400
    assert response.json()["detail"] == "ID token not found in Google response."

@pytest.mark.asyncio
@patch("httpx.AsyncClient.post")
@patch("jwt.decode")
async def test_google_callback_missing_email_or_sub_in_id_token(mock_jwt_decode, mock_httpx_post, client: TestClient):
    mock_httpx_post.return_value = AsyncMock(
        json=lambda: {
            "id_token": "mock_id_token"
        },
        raise_for_status=lambda: None
    )
    mock_jwt_decode.return_value = {
        "name": "User Only" # Missing email and sub
    }
    response = client.get("/api/v1/auth/google/callback?code=mock_code")
    assert response.status_code == 400
    assert response.json()["detail"] == "Email or Google User ID not found in ID token."

@pytest.mark.asyncio
async def test_google_auth_missing_env_vars(client: TestClient):
    with patch.dict(os.environ, {
        "GOOGLE_CLIENT_ID": "", # Simulate missing
        "GOOGLE_REDIRECT_URI": "" # Simulate missing
    }, clear=True):
        os.environ.pop("GOOGLE_CLIENT_ID", None)
        os.environ.pop("GOOGLE_REDIRECT_URI", None)
        response = client.get("/api/v1/auth/google")
        assert response.status_code == 500
        assert response.json()["detail"] == "Google OAuth environment variables not set."

@pytest.mark.asyncio
@patch("httpx.AsyncClient.post")
async def test_google_callback_missing_env_vars(mock_httpx_post, client: TestClient):
    with patch.dict(os.environ, {
        "GOOGLE_CLIENT_ID": "",
        "GOOGLE_CLIENT_SECRET": "",
        "GOOGLE_REDIRECT_URI": ""
    }, clear=True):
        os.environ.pop("GOOGLE_CLIENT_ID", None)
        os.environ.pop("GOOGLE_CLIENT_SECRET", None)
        os.environ.pop("GOOGLE_REDIRECT_URI", None)
        response = client.get("/api/v1/auth/google/callback?code=mock_code")
        assert response.status_code == 500
        assert response.json()["detail"] == "Google OAuth environment variables not set."
