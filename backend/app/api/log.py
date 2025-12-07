from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from typing import Optional
from uuid import UUID
from datetime import datetime

from backend.app.database import get_session
from backend.app.models.models import ChatSession, Message, Feedback # Assuming these are the models

router = APIRouter()

class LogRequest(BaseModel):
    session_id: UUID
    user_id: Optional[UUID] = None
    message_type: str # "user" or "bot"
    content: str
    mode: str # "book", "course", "selected_text"
    feedback: Optional[str] = None # "positive", "negative", "comment"

class LogResponse(BaseModel):
    status: str
    entry_id: UUID

@router.post("/log", response_model=LogResponse)
async def log_interaction_and_feedback(request: LogRequest, session: Depends(get_session)):
    try:
        # Log Message
        if request.message_type in ["user", "bot"]:
            new_message = Message(
                session_id=request.session_id,
                sender=request.message_type,
                content=request.content,
                language="en", # Defaulting to English, could be passed in request
                timestamp=datetime.utcnow()
            )
            session.add(new_message)
            session.commit()
            session.refresh(new_message)
            message_id = new_message.id
        else:
            message_id = None

        # Log Feedback if provided
        if request.feedback:
            new_feedback = Feedback(
                message_id=message_id,
                user_id=request.user_id,
                type=request.feedback,
                comment=request.content if request.feedback == "comment" else None,
                created_at=datetime.utcnow()
            )
            session.add(new_feedback)
            session.commit()
            session.refresh(new_feedback)
            entry_id = new_feedback.id
        else:
            entry_id = message_id # Use message_id as entry_id if no feedback is logged

        return LogResponse(status="logged", entry_id=entry_id)

    except Exception as e:
        session.rollback()
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=f"Failed to log interaction: {e}")
