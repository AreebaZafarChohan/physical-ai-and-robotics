import pytest
from unittest.mock import MagicMock
from sqlmodel import Session
from backend.src.db.repositories.personalization_rules import PersonalizationRuleRepository
from backend.src.db.repositories.user_profiles import UserProfileRepository
from backend.src.models.personalization_rules import PersonalizationRule
from backend.src.models.user_profile import UserProfile
from datetime import datetime


@pytest.fixture
def mock_session():
    """Mock database session"""
    session = MagicMock(spec=Session)
    session.get.return_value = None  # Default to returning None from get
    return session


@pytest.fixture
def personalization_rule_repository(mock_session):
    """Create a personalization rule repository instance"""
    return PersonalizationRuleRepository(mock_session)


@pytest.fixture
def user_profile_repository(mock_session):
    """Create a user profile repository instance"""
    return UserProfileRepository(mock_session)


def test_get_rule_by_id(personalization_rule_repository, mock_session):
    """Test getting a personalization rule by ID"""
    expected_rule = PersonalizationRule(
        id=1,
        content_path="/test/path",
        user_profile_criteria={"software_background": ["Python"]},
        content_variants={"python-focused": "content"},
        description="Test",
        priority=10,
        is_active=True
    )
    mock_session.get.return_value = expected_rule
    
    result = personalization_rule_repository.get_rule_by_id(1)
    
    assert result == expected_rule
    mock_session.get.assert_called_once_with(PersonalizationRule, 1)


def test_get_active_rules_by_path(personalization_rule_repository, mock_session):
    """Test getting active rules by content path"""
    from sqlmodel import select
    from sqlalchemy.sql import Select
    
    # Mock the exec method to return a list of rules
    mock_exec_result = MagicMock()
    mock_exec_result.all.return_value = [
        PersonalizationRule(
            id=1,
            content_path="/test/path",
            user_profile_criteria={"software_background": ["Python"]},
            content_variants={"python-focused": "content"},
            description="Test",
            priority=10,
            is_active=True
        )
    ]
    mock_session.exec.return_value = mock_exec_result
    
    result = personalization_rule_repository.get_active_rules_by_path("/test/path")
    
    # Check that exec was called with a select statement for active rules
    assert len(result) == 1
    assert result[0].id == 1
    assert mock_session.exec.called


def test_get_profile_by_user_id(user_profile_repository, mock_session):
    """Test getting a user profile by user ID"""
    from sqlmodel import select
    from sqlalchemy.sql import Select
    
    expected_profile = UserProfile(
        id=1,
        user_id=123,
        software_background=["Python"],
        hardware_background=["Raspberry Pi"],
        experience_level="beginner"
    )
    
    # Mock the exec method to return the profile
    mock_exec_result = MagicMock()
    mock_exec_result.first.return_value = expected_profile
    mock_session.exec.return_value = mock_exec_result
    
    result = user_profile_repository.get_profile_by_user_id(123)
    
    assert result == expected_profile
    # Check that exec was called with a select statement
    assert mock_session.exec.called