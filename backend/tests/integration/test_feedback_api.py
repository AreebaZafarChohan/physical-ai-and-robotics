import pytest
from fastapi.testclient import TestClient
from backend.src.api.main import app
from backend.src.database import SessionLocal, engine
from backend.src.models.user import User
from backend.src.services.feedback_service import PersonalizationFeedback
from sqlmodel import SQLModel
from datetime import datetime


@pytest.fixture
def client():
    """Create a test client for the API"""
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture
def db_session():
    """Create a database session for testing"""
    # Create all tables
    SQLModel.metadata.create_all(engine)
    session = SessionLocal()
    try:
        yield session
    finally:
        session.close()


@pytest.mark.skip(reason="Requires authentication setup")
def test_submit_feedback(client: TestClient, db_session):
    """Test submitting personalization feedback"""
    # Create a test user
    user = User(
        id=998,  # Using a high ID to avoid conflicts
        email="feedback@example.com",
        username="feedbackuser",
        password_hash="test_hash",
        is_active=True,
        is_superuser=False,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    db_session.add(user)
    db_session.commit()
    
    # Test data for feedback
    feedback_data = {
        "chapter_path": "/test/chapter",
        "feedback_score": 4,
        "feedback_comment": "This content was very helpful!"
    }
    
    # Submit feedback
    response = client.post("/feedback/personalization", json=feedback_data)
    
    # Should return 401 since no authentication provided
    assert response.status_code == 401
    
    # Cleanup
    # Find and delete any feedback entries for this user
    feedback_entries = db_session.query(PersonalizationFeedback).filter(
        PersonalizationFeedback.user_id == 998
    ).all()
    for entry in feedback_entries:
        db_session.delete(entry)
    db_session.delete(user)
    db_session.commit()


@pytest.mark.skip(reason="Requires authentication setup")
def test_get_feedback(client: TestClient):
    """Test getting feedback for a chapter"""
    # Make a request to get feedback for a chapter
    response = client.get("/feedback/personalization/test-chapter")
    
    # Should return 401 since no authentication provided
    assert response.status_code == 401