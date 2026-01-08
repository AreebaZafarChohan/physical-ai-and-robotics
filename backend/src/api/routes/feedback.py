from typing import Optional
from fastapi import APIRouter, Depends, HTTPException, status
from sqlmodel import select
from sqlalchemy.ext.asyncio import AsyncSession

from backend.src.database import get_session
from backend.src.models.user import User
from backend.src.services.user_service import get_current_user
from backend.src.services.feedback_service import FeedbackService, PersonalizationFeedback


router = APIRouter(prefix="/feedback", tags=["feedback"])


@router.post("/personalization")
async def submit_personalization_feedback(
    chapter_path: str,
    feedback_score: int,  # 1-5 rating
    feedback_comment: Optional[str] = None,
    current_user: User = Depends(get_current_user),
    session: AsyncSession = Depends(get_session)
):
    """
    Submit feedback about personalization relevance
    """
    if feedback_score < 1 or feedback_score > 5:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Feedback score must be between 1 and 5"
        )
    
    feedback_service = FeedbackService(session)

    try:
        feedback = await feedback_service.submit_feedback(
            user_id=current_user.id,
            chapter_path=chapter_path,
            feedback_score=feedback_score,
            feedback_comment=feedback_comment
        )
        
        return {
            "message": "Feedback submitted successfully",
            "feedback_id": feedback.id
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error submitting feedback: {str(e)}"
        )


@router.get("/personalization/{chapter_path}")
async def get_personalization_feedback(
    chapter_path: str,
    current_user: User = Depends(get_current_user),  # Could make this public depending on requirements
    session: AsyncSession = Depends(get_session)
):
    """
    Get feedback statistics for a specific chapter
    """
    feedback_service = FeedbackService(session)

    try:
        avg_score = await feedback_service.get_average_feedback_score(chapter_path)
        feedback_list = await feedback_service.get_feedback_for_chapter(chapter_path)
        
        return {
            "chapter_path": chapter_path,
            "average_score": avg_score,
            "total_feedbacks": len(feedback_list),
            "feedbacks": [
                {
                    "id": fb.id,
                    "score": fb.feedback_score,
                    "comment": fb.feedback_comment,
                    "created_at": fb.created_at
                }
                for fb in feedback_list
            ]
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving feedback: {str(e)}"
        )