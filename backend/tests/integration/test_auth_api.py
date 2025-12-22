import pytest
from fastapi.testclient import TestClient
from sqlmodel import Session, create_engine, SQLModel
from backend.src.api.main import app # Import your main FastAPI app
from backend.src.models.user import User
from backend.src.models.personalization import PersonalizationData
from backend.src.database import get_session, create_db_and_tables

# Use a separate test database
TEST_DATABASE_URL = "sqlite:///./test.db"
engine = create_engine(TEST_DATABASE_URL, echo=True)

def create_test_db_and_tables():
    SQLModel.metadata.create_all(engine)

@pytest.fixture(name="session")
def session_fixture():
    create_test_db_and_tables()
    with Session(engine) as session:
        yield session
    SQLModel.metadata.drop_all(engine) # Clean up after tests

@pytest.fixture(name="client")
def client_fixture(session: Session):
    def get_session_override():
        return session

    app.dependency_overrides[get_session] = get_session_override
    client = TestClient(app)
    yield client
    app.dependency_overrides.clear()

def test_signup_email_password_success(client: TestClient):
    response = client.post(
        "/auth/signup",
        json={
            "email": "test@example.com",
            "password": "Password123",
            "software_background": ["Python", "FastAPI"],
            "hardware_background": ["Raspberry Pi"]
        }
    )
    assert response.status_code == 200
    assert "access_token" in response.json()
    assert response.json()["token_type"] == "bearer"

def test_signup_email_password_already_exists(client: TestClient):
    client.post(
        "/auth/signup",
        json={
            "email": "existing@example.com",
            "password": "Password123",
        }
    )
    response = client.post(
        "/auth/signup",
        json={
            "email": "existing@example.com",
            "password": "Password123",
        }
    )
    assert response.status_code == 409
    assert response.json()["detail"] == "Email already registered"

def test_login_success(client: TestClient):
    client.post(
        "/auth/signup",
        json={
            "email": "login@example.com",
            "password": "Password123",
        }
    )
    response = client.post(
        "/auth/login",
        data={"username": "login@example.com", "password": "Password123"}
    )
    assert response.status_code == 200
    assert "access_token" in response.json()

def test_login_invalid_credentials(client: TestClient):
    response = client.post(
        "/auth/login",
        data={"username": "nonexistent@example.com", "password": "WrongPassword"}
    )
    assert response.status_code == 401
    assert response.json()["detail"] == "Incorrect email or password"

def test_get_user_profile(client: TestClient):
    signup_response = client.post(
        "/auth/signup",
        json={
            "email": "profile@example.com",
            "password": "Password123",
        }
    )
    token = signup_response.json()["access_token"]
    response = client.get(
        "/user/profile",
        headers={"Authorization": f"Bearer {token}"}
    )
    assert response.status_code == 200
    assert response.json()["email"] == "profile@example.com"

def test_get_personalization_data(client: TestClient):
    signup_response = client.post(
        "/auth/signup",
        json={
            "email": "personalize@example.com",
            "password": "Password123",
            "software_background": ["Python"],
            "hardware_background": ["Arduino"]
        }
    )
    token = signup_response.json()["access_token"]
    response = client.get(
        "/user/personalization-data",
        headers={"Authorization": f"Bearer {token}"}
    )
    assert response.status_code == 200
    assert "software_background" in response.json()
    assert "hardware_background" in response.json()
    assert "Python" in response.json()["software_background"]

def test_logout(client: TestClient):
    signup_response = client.post(
        "/auth/signup",
        json={
            "email": "logout@example.com",
            "password": "Password123",
        }
    )
    token = signup_response.json()["access_token"]
    response = client.post(
        "/auth/logout",
        headers={"Authorization": f"Bearer {token}"}
    )
    assert response.status_code == 200
    assert response.json()["message"] == "Successfully logged out"
