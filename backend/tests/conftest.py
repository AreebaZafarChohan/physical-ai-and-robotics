# backend/tests/conftest.py
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from sqlmodel import SQLModel, Session, create_engine
from sqlalchemy.pool import StaticPool

from backend.src.database import get_session
from backend.src.models.user import User

# --- Test Database Setup (Synchronous SQLite) ---
SQLALCHEMY_DATABASE_URL = "sqlite:///:memory:"
engine = create_engine(
    SQLALCHEMY_DATABASE_URL,
    connect_args={"check_same_thread": False},
    poolclass=StaticPool,
)

@pytest.fixture(name="session")
def session_fixture():
    SQLModel.metadata.create_all(engine)
    with Session(engine) as session:
        yield session
    SQLModel.metadata.drop_all(engine)

# --- Fixture for a test client that uses a dedicated test app ---
@pytest.fixture(name="client")
def client_fixture(session: Session):
    from backend.src.api.auth import router as auth_router

    test_app = FastAPI()

    # Override the get_session dependency for this test app
    def override_get_session():
        yield session

    test_app.dependency_overrides[get_session] = override_get_session

    test_app.include_router(auth_router, prefix="/auth", tags=["Auth"])

    with TestClient(test_app) as client:
        yield client
    test_app.dependency_overrides.clear()