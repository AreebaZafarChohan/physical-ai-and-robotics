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