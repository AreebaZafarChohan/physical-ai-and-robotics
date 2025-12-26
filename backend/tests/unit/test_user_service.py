import pytest
from unittest.mock import MagicMock
from sqlmodel import Session
from backend.src.services.user_service import UserService
from backend.src.models.user_profile import UserProfile


@pytest.fixture
def mock_session():
    """Mock database session"""
    return MagicMock(spec=Session)


@pytest.fixture
def user_service(mock_session):
    """Create a user service instance with mocked session"""
    return UserService(mock_session)


def test_validate_user_profile_for_personalization_valid(user_service):
    """Test validating a valid user profile for personalization"""
    user_profile = {
        "software_background": ["Python"],
        "hardware_background": ["Raspberry Pi"],
        "experience_level": "intermediate"
    }
    
    is_valid, message = user_service.validate_user_profile_for_personalization(user_profile)
    
    assert is_valid is True
    assert message == "Profile is valid for personalization"


def test_validate_user_profile_for_personalization_invalid_level(user_service):
    """Test validating a user profile with invalid experience level"""
    user_profile = {
        "software_background": ["Python"],
        "hardware_background": ["Raspberry Pi"],
        "experience_level": "invalid_level"
    }
    
    is_valid, message = user_service.validate_user_profile_for_personalization(user_profile)
    
    assert is_valid is False
    assert "Invalid experience level" in message


def test_validate_user_profile_for_personalization_no_background(user_service):
    """Test validating a user profile with no background information"""
    user_profile = {
        "software_background": [],
        "hardware_background": [],
        "experience_level": "beginner"
    }
    
    is_valid, message = user_service.validate_user_profile_for_personalization(user_profile)
    
    assert is_valid is False
    assert "must contain software or hardware background information" in message


def test_validate_user_profile_for_personalization_none(user_service):
    """Test validating a None user profile"""
    is_valid, message = user_service.validate_user_profile_for_personalization(None)
    
    assert is_valid is False
    assert "not found" in message