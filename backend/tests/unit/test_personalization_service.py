import pytest
from unittest.mock import AsyncMock, MagicMock
from sqlmodel import Session
from backend.src.services.personalization_service import PersonalizationService
from backend.src.models.user_profile import UserProfile
from backend.src.models.personalization_rules import PersonalizationRule


@pytest.fixture
def mock_session():
    """Mock database session"""
    return MagicMock(spec=Session)


@pytest.fixture
def personalization_service(mock_session):
    """Create a personalization service instance with mocked session"""
    return PersonalizationService(mock_session)


@pytest.mark.asyncio
async def test_get_personalized_content_with_valid_profile(personalization_service):
    """Test getting personalized content with a valid user profile"""
    # Create a mock user profile
    user_profile = UserProfile(
        id=1,
        user_id=123,
        software_background=["Python", "JavaScript"],
        hardware_background=["Raspberry Pi"],
        experience_level="intermediate"
    )
    
    # Mock the content service to return some content
    original_content = "<p>Original content</p>{{personalize:python}}Python content{{personalize:end}}"
    personalization_service.get_original_content = AsyncMock(return_value=original_content)
    
    # Mock the content parser to return personalized content
    from backend.src.utils.content_parser import adapt_content_by_user_profile
    # Since we can't easily mock this function, we'll check that it's called appropriately
    # Just test the flow through the service
    
    # Mock cache methods
    personalization_service._get_cached_personalized_content = AsyncMock(return_value=None)
    personalization_service._cache_personalized_content = AsyncMock()
    personalization_service._apply_personalization_rules = AsyncMock(return_value="<p>Personalized content</p>Python content")
    
    result = await personalization_service.get_personalized_content("/test/chapter", user_profile)
    
    # Verify the result is what we expect
    assert result == "<p>Personalized content</p>Python content"
    
    # Verify cache methods were called
    personalization_service._get_cached_personalized_content.assert_called_once()
    personalization_service._cache_personalized_content.assert_called_once()


@pytest.mark.asyncio
async def test_get_personalized_content_with_invalid_profile(personalization_service):
    """Test getting personalized content with an invalid user profile"""
    # Create a user profile with no background info
    user_profile = UserProfile(
        id=1,
        user_id=123,
        software_background=[],
        hardware_background=[],
        experience_level="beginner"
    )
    
    # Mock the original content
    original_content = "<p>Original content</p>"
    personalization_service.get_original_content = AsyncMock(return_value=original_content)
    
    result = await personalization_service.get_personalized_content("/test/chapter", user_profile)
    
    # Should return original content when profile is invalid for personalization
    assert result == original_content


def test_hash_user_profile(personalization_service):
    """Test the user profile hashing function"""
    user_profile = UserProfile(
        id=1,
        user_id=123,
        software_background=["Python", "JavaScript"],
        hardware_background=["Raspberry Pi"],
        experience_level="intermediate"
    )
    
    hash1 = personalization_service._hash_user_profile(user_profile)
    # Create another profile with same data but different order
    user_profile2 = UserProfile(
        id=1,
        user_id=123,
        software_background=["JavaScript", "Python"],  # Different order
        hardware_background=["Raspberry Pi"],
        experience_level="intermediate"
    )
    
    hash2 = personalization_service._hash_user_profile(user_profile2)
    
    # Should be the same hash since software backgrounds are the same
    assert hash1 == hash2


def test_rule_matches_profile(personalization_service):
    """Test the rule matching logic"""
    # Create a rule that matches Python users with beginner experience
    rule = PersonalizationRule(
        content_path="/test/chapter",
        user_profile_criteria={
            "software_background": ["Python"],
            "experience_level": ["beginner"]
        },
        content_variants={},
        description="Test rule",
        priority=10,
        is_active=True
    )
    
    # Create a user profile that matches
    user_profile = UserProfile(
        id=1,
        user_id=123,
        software_background=["Python", "JavaScript"],
        hardware_background=["Raspberry Pi"],
        experience_level="beginner"
    )
    
    matches = personalization_service._rule_matches_profile(rule, user_profile)
    assert matches is True
    
    # Create a user profile that doesn't match
    user_profile2 = UserProfile(
        id=2,
        user_id=124,
        software_background=["JavaScript"],
        hardware_background=["Arduino"],
        experience_level="advanced"
    )
    
    matches2 = personalization_service._rule_matches_profile(rule, user_profile2)
    assert matches2 is False