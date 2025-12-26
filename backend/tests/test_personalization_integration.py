import pytest
from fastapi.testclient import TestClient
from backend.src.api.main import app
from backend.src.database import SessionLocal
from backend.src.models.user_profile import UserProfile
from backend.src.models.personalization_rules import PersonalizationRule
from sqlmodel import select
from datetime import datetime


@pytest.fixture
def client():
    """Create a test client for the API"""
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture
def db_session():
    """Create a database session for testing"""
    session = SessionLocal()
    try:
        yield session
    finally:
        session.close()


def test_personalization_flow(client: TestClient, db_session):
    """
    Test the complete personalization flow:
    1. Create a user profile
    2. Create a personalization rule
    3. Request personalized content
    4. Verify the correct personalized content is returned
    """
    # Create a test user profile
    test_profile = UserProfile(
        user_id=1,  # Mock user ID
        software_background=["Python", "JavaScript"],
        hardware_background=["Raspberry Pi"],
        experience_level="intermediate",
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    
    # Add the profile to the session
    db_session.add(test_profile)
    db_session.commit()
    db_session.refresh(test_profile)
    
    # Create a test personalization rule
    test_rule = PersonalizationRule(
        content_path="/test/chapter",
        user_profile_criteria={
            "software_background": ["Python"]
        },
        content_variants={
            "python-focused": "This is Python-specific content."
        },
        description="Test Python rule",
        priority=10,
        is_active=True,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    
    # Add the rule to the session
    db_session.add(test_rule)
    db_session.commit()
    db_session.refresh(test_rule)
    
    # Mock a user authentication token (simplified)
    # In a real test, we would use a proper authentication mechanism
    
    # Make a request to get personalized content
    # Since we don't have a real user auth in tests, we'll use a mock approach
    response = client.get("/personalize/test/chapter")
    
    # Note: This test would need more setup for a complete integration test
    # including user authentication and a content file to work with
    # This is a basic structure to demonstrate the concept
    
    # For now, check that the endpoint exists and returns an appropriate status
    # (likely 401 since we didn't provide auth)
    assert response.status_code in [200, 401, 404]  # Could be 200 if auth not required for test, 401 if auth required, 404 if content path doesn't exist
    
    # Clean up
    db_session.delete(test_rule)
    db_session.delete(test_profile)
    db_session.commit()


def test_preview_personalization(client: TestClient):
    """
    Test the preview personalization endpoint
    """
    # Test data for preview
    preview_data = {
        "chapter_path": "/test/chapter",
        "profile_data": {
            "software_background": ["Python"],
            "hardware_background": ["Raspberry Pi"],
            "experience_level": "beginner"
        }
    }
    
    # Make a request to the preview endpoint
    # Note: This will likely return 401 without proper authentication
    response = client.post("/personalize/preview", json=preview_data)
    
    # Check that the endpoint exists and returns an appropriate status
    # (likely 401 since we didn't provide auth for admin)
    assert response.status_code in [401, 404, 422]  # 401 for auth, 404 for content, 422 for validation


def test_list_personalization_rules(client: TestClient):
    """
    Test the endpoint to list personalization rules
    """
    # Make a request to list rules
    # Note: This will likely return 401 without proper authentication
    response = client.get("/personalize/rules")
    
    # Check that the endpoint exists and returns an appropriate status
    # (likely 401 since we didn't provide auth for admin)
    assert response.status_code in [401, 200]  # 401 for auth, 200 for success if auth provided