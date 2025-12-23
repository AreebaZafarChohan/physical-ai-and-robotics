import pytest
from fastapi.testclient import TestClient
from sqlmodel import Session, create_engine, SQLModel
from backend.src.api.main import app
from backend.src.models.user import User
from backend.src.database import get_session, create_db_and_tables
from backend.src.utils.security import get_password_hash

# Use a separate test database
TEST_DATABASE_URL = "sqlite:///./test_performance.db"
engine = create_engine(TEST_DATABASE_URL, echo=True)

def create_test_db_and_tables():
    SQLModel.metadata.create_all(engine)

@pytest.fixture(name="session_performance")
def session_performance_fixture():
    create_test_db_and_tables()
    with Session(engine) as session:
        yield session
    SQLModel.metadata.drop_all(engine)

@pytest.fixture(name="client_performance")
def client_performance_fixture(session_performance: Session):
    def get_session_override():
        return session_performance

    app.dependency_overrides[get_session] = get_session_override
    client = TestClient(app)
    yield client
    app.dependency_overrides.clear()

@pytest.fixture(autouse=True, scope="module")
def setup_users_for_performance(client_performance: TestClient):
    # Setup some users for login performance testing
    with Session(engine) as session:
        for i in range(100):
            user = User(email=f"user_{i}@perf.com", password_hash=get_password_hash(f"password{i}"))
            session.add(user)
        session.commit()

def test_performance_signup(benchmark, client_performance: TestClient):
    email_counter = 0
    def signup_request():
        nonlocal email_counter
        email_counter += 1
        return client_performance.post(
            "/auth/signup",
            json={
                "email": f"perf_test_signup_{email_counter}@example.com",
                "password": "Password123",
            }
        )
    benchmark(signup_request)

def test_performance_login(benchmark, client_performance: TestClient):
    def login_request():
        return client_performance.post(
            "/auth/login",
            data={"username": "user_1@perf.com", "password": "password1"}
        )
    benchmark(login_request)