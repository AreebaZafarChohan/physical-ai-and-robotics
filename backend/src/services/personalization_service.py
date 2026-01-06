<<<<<<< HEAD
import asyncio
import json
import logging
from typing import Dict, List, Optional
from sqlmodel import Session, select
from backend.src.models.personalization_rules import PersonalizationRule
from backend.src.models.user_profile import UserProfile
from backend.src.models.personalization_cache import PersonalizationCache
from backend.src.utils.content_parser import parse_content_with_markers  # We'll create this utility
from datetime import datetime, timedelta
import hashlib

# Set up logger for personalization service
logger = logging.getLogger(__name__)


class PersonalizationService:
    """
    Service class for personalization-related operations
    """
    
    def __init__(self, session: Session):
        self.session = session
        self.cache_ttl = timedelta(hours=24)  # 24-hour cache TTL
    
    async def get_personalized_content(
        self,
        chapter_path: str,
        user_profile: UserProfile
    ) -> str:
        """
        Get personalized content for a chapter based on user profile
        """
        start_time = datetime.utcnow()
        logger.info(f"Starting personalization request for chapter: {chapter_path}, user_id: {user_profile.user_id}")

        try:
            # Validate user profile for personalization
            from backend.src.services.user_service import UserService
            user_service = UserService(self.session)
            is_valid, validation_message = user_service.validate_user_profile_for_personalization({
                "software_background": user_profile.software_background,
                "hardware_background": user_profile.hardware_background,
                "experience_level": user_profile.experience_level
            })

            if not is_valid:
                # If profile is not valid for personalization, return original content
                logger.warning(f"User profile not valid for personalization: {validation_message}, user_id: {user_profile.user_id}")
                original_content = await self.get_original_content(chapter_path)
                logger.info(f"Returning original content for chapter {chapter_path} due to invalid profile")
                return original_content

            # Create a hash of user profile for caching
            profile_hash = self._hash_user_profile(user_profile)
            logger.debug(f"Profile hash generated for user_id: {user_profile.user_id}, hash: {profile_hash}")

            # Try to get content from cache first
            cached_content = await self._get_cached_personalized_content(profile_hash, chapter_path)

            if cached_content:
                logger.info(f"Cache hit for chapter {chapter_path}, user_id: {user_profile.user_id}")
                return cached_content
            else:
                logger.info(f"Cache miss for chapter {chapter_path}, user_id: {user_profile.user_id}")

            # Fetch original content (this would come from the Docusaurus content system)
            original_content = await self.get_original_content(chapter_path)
            logger.debug(f"Fetched original content for chapter {chapter_path}")

            # Apply personalization rules to the content
            personalized_content = await self._apply_personalization_rules(
                original_content,
                user_profile,
                chapter_path
            )

            # If no personalization was applied (content is the same as original), return original
            if personalized_content == original_content:
                logger.info(f"No personalization rules applied for chapter {chapter_path}, user_id: {user_profile.user_id}")
            else:
                logger.info(f"Personalization applied for chapter {chapter_path}, user_id: {user_profile.user_id}")

            # Cache the result
            await self._cache_personalized_content(
                profile_hash,
                chapter_path,
                personalized_content
            )
            logger.debug(f"Content cached for chapter {chapter_path}, user_id: {user_profile.user_id}")

            processing_time = (datetime.utcnow() - start_time).total_seconds()
            logger.info(f"Personalization completed for chapter {chapter_path}, processing_time: {processing_time}s, user_id: {user_profile.user_id}")

            return personalized_content
        except Exception as e:
            # Log the error with details
            processing_time = (datetime.utcnow() - start_time).total_seconds()
            logger.error(f"Error in get_personalized_content for chapter {chapter_path}, user_id: {user_profile.user_id}, processing_time: {processing_time}s, error: {str(e)}")

            # For graceful degradation, return original content if personalization fails
            try:
                original_content = await self.get_original_content(chapter_path)
                logger.warning(f"Returning original content due to error for chapter {chapter_path}")
                return original_content
            except:
                # If we can't get the original content either, return a default message
                logger.error(f"Could not fetch original content for chapter {chapter_path}")
                return f"Content for {chapter_path} is temporarily unavailable. Please try again later."
    
    async def get_original_content(self, chapter_path: str) -> str:
        """
        Get original chapter content
        """
        from pathlib import Path
        from backend.src.services.content_service import ContentService

        # Resolve docs base path consistently with the main API route
        BASE_DIR = Path(__file__).resolve().parents[4]  # Go up to project root
        DOCS_PATH = BASE_DIR / "frontend" / "docs"

        content_service = ContentService(content_base_path=DOCS_PATH)
        return await content_service.get_content_by_path(chapter_path)
    
    async def _apply_personalization_rules(
        self, 
        content: str, 
        user_profile: UserProfile, 
        chapter_path: str
    ) -> str:
        """
        Apply personalization rules to content based on user profile
        """
        # Fetch applicable personalization rules for this chapter and user profile
        rules = await self._get_applicable_rules(chapter_path, user_profile)
        
        # Sort rules by priority (lower numbers have higher priority)
        rules.sort(key=lambda rule: rule.priority)
        
        # Apply each rule to the content
        personalized_content = content
        
        for rule in rules:
            # Apply the rule if it matches the user's profile
            if self._rule_matches_profile(rule, user_profile):
                # Apply the content replacement based on the rule
                personalized_content = self._apply_rule_to_content(
                    personalized_content, 
                    rule, 
                    user_profile
                )
        
        return personalized_content
    
    async def _get_applicable_rules(
        self, 
        chapter_path: str, 
        user_profile: UserProfile
    ) -> List[PersonalizationRule]:
        """
        Get personalization rules that apply to the given chapter path and user profile
        """
        statement = select(PersonalizationRule).where(
            (PersonalizationRule.content_path == chapter_path) &
            (PersonalizationRule.is_active == True)
        )
        
        rules = self.session.exec(statement).all()
        
        # Filter rules that are applicable to the user's profile
        applicable_rules = []
        for rule in rules:
            if self._rule_matches_profile(rule, user_profile):
                applicable_rules.append(rule)
        
        return applicable_rules
    
    def _rule_matches_profile(
        self,
        rule: PersonalizationRule,
        user_profile: UserProfile
    ) -> bool:
        """
        Check if a personalization rule matches the user profile
        """
        criteria = rule.user_profile_criteria

        # Check experience level matches if specified in criteria
        if "experience_level" in criteria:
            if isinstance(criteria["experience_level"], list):
                if user_profile.experience_level not in criteria["experience_level"]:
                    return False
            else:
                # If it's a string, compare directly
                if user_profile.experience_level != criteria["experience_level"]:
                    return False

        # Check software background matches if specified in criteria
        if "software_background" in criteria:
            user_software_set = set(user_profile.software_background)
            if isinstance(criteria["software_background"], list):
                criteria_software_set = set(criteria["software_background"])
            else:
                # If it's a string, wrap in a set
                criteria_software_set = {criteria["software_background"]}

            if not user_software_set.intersection(criteria_software_set):
                return False

        # Check hardware background matches if specified in criteria
        if "hardware_background" in criteria:
            user_hardware_set = set(user_profile.hardware_background)
            if isinstance(criteria["hardware_background"], list):
                criteria_hardware_set = set(criteria["hardware_background"])
            else:
                # If it's a string, wrap in a set
                criteria_hardware_set = {criteria["hardware_background"]}

            if not user_hardware_set.intersection(criteria_hardware_set):
                return False

        # Additional criteria can be checked here

        return True
    
    def _apply_rule_to_content(
        self,
        content: str,
        rule: PersonalizationRule,
        user_profile: UserProfile
    ) -> str:
        """
        Apply a personalization rule to content
        """
        from backend.src.utils.content_parser import adapt_content_by_user_profile

        # Use the enhanced content adaptation based on user profile
        return adapt_content_by_user_profile(content, {
            "software_background": user_profile.software_background,
            "hardware_background": user_profile.hardware_background,
            "experience_level": user_profile.experience_level
        })
    
    def _variant_applicable_to_profile(
        self, 
        variant_type: str, 
        user_profile: UserProfile
    ) -> bool:
        """
        Check if a content variant is applicable to the user's profile
        """
        # Check if variant type matches experience level
        if variant_type == user_profile.experience_level:
            return True
        
        # Check if variant type matches any of the user's software backgrounds
        for sb in user_profile.software_background:
            if variant_type.lower() in sb.lower().replace(' ', '-').replace(' ', '_'):
                return True
        
        # Check if variant type matches any of the user's hardware backgrounds
        for hb in user_profile.hardware_background:
            if variant_type.lower() in hb.lower().replace(' ', '-').replace(' ', '_'):
                return True
        
        return False
    
    async def _get_cached_personalized_content(
        self, 
        profile_hash: str, 
        content_path: str
    ) -> Optional[str]:
        """
        Get cached personalized content if it exists and hasn't expired
        """
        statement = select(PersonalizationCache).where(
            (PersonalizationCache.user_profile_hash == profile_hash) &
            (PersonalizationCache.content_path == content_path) &
            (PersonalizationCache.cache_expires_at > datetime.utcnow())
        )
        
        cached_item = self.session.exec(statement).first()
        
        if cached_item:
            return cached_item.personalized_content
        
        return None
    
    async def _cache_personalized_content(
        self, 
        profile_hash: str, 
        content_path: str, 
        content: str
    ) -> None:
        """
        Cache personalized content with TTL
        """
        # Check if there's already a cache entry for this user-profile/content combination
        statement = select(PersonalizationCache).where(
            (PersonalizationCache.user_profile_hash == profile_hash) &
            (PersonalizationCache.content_path == content_path)
        )
        
        existing_cache = self.session.exec(statement).first()
        
        if existing_cache:
            # Update the existing cache entry
            existing_cache.personalized_content = content
            existing_cache.cache_created_at = datetime.utcnow()
            existing_cache.cache_expires_at = datetime.utcnow() + self.cache_ttl
            self.session.add(existing_cache)
        else:
            # Create a new cache entry
            cache_entry = PersonalizationCache(
                user_profile_hash=profile_hash,
                content_path=content_path,
                personalized_content=content,
                cache_created_at=datetime.utcnow(),
                cache_expires_at=datetime.utcnow() + self.cache_ttl
            )
            self.session.add(cache_entry)
        
        self.session.commit()
    
    def _hash_user_profile(self, user_profile: UserProfile) -> str:
        """
        Create a hash of the user profile for caching purposes
        """
        profile_data = {
            "software_background": sorted(user_profile.software_background),
            "hardware_background": sorted(user_profile.hardware_background),
            "experience_level": user_profile.experience_level
        }
        
        profile_str = json.dumps(profile_data, sort_keys=True)
        return hashlib.sha256(profile_str.encode()).hexdigest()
=======
from typing import Optional, Dict, Any, Union
from sqlmodel import SQLModel, Field, Session, select
from datetime import datetime
from backend.src.models.user import User
import re


class PersonalizationFeedback(SQLModel, table=True):
    """
    Model for collecting user feedback on personalization relevance
    """
    __tablename__ = "personalization_feedback"

    id: Optional[int] = Field(default=None, primary_key=True)
    user_id: int = Field(foreign_key="user.id")
    chapter_path: str = Field(max_length=500)
    feedback_score: int = Field(ge=1, le=5)  # 1-5 rating
    feedback_comment: Optional[str] = Field(default=None, max_length=1000)
    created_at: datetime = Field(default_factory=datetime.utcnow)


class FeedbackService:
    """
    Service for handling personalization feedback
    """

    def __init__(self, session: Session):
        self.session = session

    def submit_feedback(
        self,
        user_id: int,
        chapter_path: str,
        feedback_score: int,
        feedback_comment: Optional[str] = None
    ) -> PersonalizationFeedback:
        """
        Submit feedback for personalization relevance
        """
        feedback = PersonalizationFeedback(
            user_id=user_id,
            chapter_path=chapter_path,
            feedback_score=feedback_score,
            feedback_comment=feedback_comment
        )

        self.session.add(feedback)
        self.session.commit()
        self.session.refresh(feedback)

        return feedback

    def get_feedback_for_chapter(self, chapter_path: str) -> list[PersonalizationFeedback]:
        """
        Get all feedback for a specific chapter
        """
        statement = select(PersonalizationFeedback).where(
            PersonalizationFeedback.chapter_path == chapter_path
        )
        return self.session.exec(statement).all()

    def get_average_feedback_score(self, chapter_path: str) -> Optional[float]:
        """
        Get the average feedback score for a chapter
        """
        feedback_list = self.get_feedback_for_chapter(chapter_path)
        if not feedback_list:
            return None

        total_score = sum(f.feedback_score for f in feedback_list)
        return total_score / len(feedback_list)


class PersonalizationService:
    """
    Service for handling content personalization based on user profiles
    """

    def __init__(self):
        # Define content markers for different user types
        self.markers = {
            'beginner': {
                'pattern': r'\{\{personalize:beginner\}\}([\s\S]*?)\{\{personalize:end\}\}',
                'replacement': r'\1'
            },
            'intermediate': {
                'pattern': r'\{\{personalize:intermediate\}\}([\s\S]*?)\{\{personalize:end\}\}',
                'replacement': r'\1'
            },
            'advanced': {
                'pattern': r'\{\{personalize:advanced\}\}([\s\S]*?)\{\{personalize:end\}\}',
                'replacement': r'\1'
            },
            'software_background': {
                'pattern': r'\{\{personalize:software\}\}([\s\S]*?)\{\{personalize:end\}\}',
                'replacement': r'\1'
            },
            'hardware_background': {
                'pattern': r'\{\{personalize:hardware\}\}([\s\S]*?)\{\{personalize:end\}\}',
                'replacement': r'\1'
            }
        }

    async def personalize_content(
        self,
        content: str,
        user_profile: Optional[Union[Dict[str, Any], 'UserProfile']] = None
    ) -> str:
        """
        Apply personalization to content based on user profile
        """
        if not user_profile:
            # If no user profile, return content without personalization markers
            return self._remove_personalization_markers(content)

        # Convert user_profile to dict if it's an object
        profile_dict = user_profile
        if hasattr(user_profile, '__dict__'):
            profile_dict = user_profile.__dict__
        elif hasattr(user_profile, '_sa_instance_state'):
            # If it's a SQLAlchemy/SQLModel object, convert to dict
            profile_dict = {c.name: getattr(user_profile, c.name) for c in user_profile.__table__.columns}

        # Apply personalization based on profile
        personalized_content = content

        # Apply experience level personalization
        experience_level = profile_dict.get('experience_level', 'beginner').lower()
        if experience_level:
            personalized_content = self._apply_experience_personalization(
                personalized_content,
                experience_level
            )

        # Apply background personalization
        software_background = profile_dict.get('software_background', [])
        hardware_background = profile_dict.get('hardware_background', [])

        if software_background:
            personalized_content = self._apply_background_personalization(
                personalized_content,
                'software'
            )

        if hardware_background:
            personalized_content = self._apply_background_personalization(
                personalized_content,
                'hardware'
            )

        # Remove any remaining unprocessed personalization markers
        personalized_content = self._remove_personalization_markers(personalized_content)

        return personalized_content

    def _apply_experience_personalization(self, content: str, experience_level: str) -> str:
        """
        Apply personalization based on experience level
        """
        import re

        # Replace markers based on experience level
        if experience_level == 'beginner':
            # Show beginner content, hide advanced content
            content = re.sub(
                r'\{\{personalize:advanced\}\}[\s\S]*?\{\{personalize:end\}\}',
                '',
                content
            )
            content = re.sub(
                r'\{\{personalize:beginner\}\}([\s\S]*?)\{\{personalize:end\}\}',
                r'\1',
                content
            )
            content = re.sub(
                r'\{\{personalize:intermediate\}\}([\s\S]*?)\{\{personalize:end\}\}',
                '',
                content
            )
        elif experience_level == 'intermediate':
            # Show intermediate content, hide beginner/advanced as appropriate
            content = re.sub(
                r'\{\{personalize:beginner\}\}[\s\S]*?\{\{personalize:end\}\}',
                '',
                content
            )
            content = re.sub(
                r'\{\{personalize:intermediate\}\}([\s\S]*?)\{\{personalize:end\}\}',
                r'\1',
                content
            )
        elif experience_level == 'advanced':
            # Show advanced content, hide beginner content
            content = re.sub(
                r'\{\{personalize:beginner\}\}[\s\S]*?\{\{personalize:end\}\}',
                '',
                content
            )
            content = re.sub(
                r'\{\{personalize:intermediate\}\}([\s\S]*?)\{\{personalize:end\}\}',
                '',
                content
            )
            content = re.sub(
                r'\{\{personalize:advanced\}\}([\s\S]*?)\{\{personalize:end\}\}',
                r'\1',
                content
            )

        return content

    def _apply_background_personalization(self, content: str, background_type: str) -> str:
        """
        Apply personalization based on user background
        """
        import re

        if background_type == 'software':
            # Show software-focused content
            content = re.sub(
                r'\{\{personalize:software\}\}([\s\S]*?)\{\{personalize:end\}\}',
                r'\1',
                content
            )
            # Optionally hide hardware-focused content if user is software-focused
            content = re.sub(
                r'\{\{personalize:hardware\}\}[\s\S]*?\{\{personalize:end\}\}',
                '',
                content
            )
        elif background_type == 'hardware':
            # Show hardware-focused content
            content = re.sub(
                r'\{\{personalize:hardware\}\}([\s\S]*?)\{\{personalize:end\}\}',
                r'\1',
                content
            )
            # Optionally hide software-focused content if user is hardware-focused
            content = re.sub(
                r'\{\{personalize:software\}\}[\s\S]*?\{\{personalize:end\}\}',
                '',
                content
            )

        return content

    def _remove_personalization_markers(self, content: str) -> str:
        """
        Remove any remaining unprocessed personalization markers
        """
        import re

        # Remove any remaining markers that weren't processed
        content = re.sub(
            r'\{\{personalize:[^}]+\}\}[\s\S]*?\{\{personalize:end\}\}',
            '',
            content
        )

        return content
>>>>>>> 012-docusaurus-i18n-urdu
