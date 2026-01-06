<<<<<<< HEAD
from typing import Optional
from sqlmodel import SQLModel, Field, Session, select
from datetime import datetime
from backend.src.models.user import User


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
=======
from typing import Optional
from sqlmodel import SQLModel, Field, Session, select
from datetime import datetime
from backend.src.models.user import User


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
>>>>>>> 012-docusaurus-i18n-urdu
        return total_score / len(feedback_list)