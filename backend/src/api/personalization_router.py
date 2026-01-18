import os
from pathlib import Path
from fastapi import APIRouter, Depends, HTTPException, Query
from typing import Optional, Dict, Any
from sqlmodel import select
from sqlalchemy.ext.asyncio import AsyncSession
from pydantic import BaseModel
from backend.src.database import get_session
from backend.src.services.content_service import ContentService
from backend.src.models.user_profile import UserProfile
from backend.src.services.personalization_service import PersonalizationService
from backend.src.models.personalization_rules import PersonalizationRule
from backend.src.models.user import User
from backend.src.api.auth import get_current_user
import logging

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/personalize", tags=["personalization"])

# Define request/response models to match frontend expectations
class PersonalizeContentRequest(BaseModel):
    chapter_path: str

class PersonalizeContentResponse(BaseModel):
    chapter_path: str
    personalized_content: str
    user_profile: Optional[Dict[str, Any]]
    personalization_applied: bool

class PreviewPersonalizationRequest(BaseModel):
    chapter_path: str
    profile_data: Dict[str, Any]

class PreviewPersonalizationResponse(BaseModel):
    chapter_path: str
    personalized_content: str
    applied_profile: Dict[str, Any]
    personalization_applied: bool

@router.get("/{chapter_path:path}", response_model=PersonalizeContentResponse)
async def get_personalized_content(
    chapter_path: str,
    current_user: Optional[User] = Depends(get_current_user),
    session: AsyncSession = Depends(get_session)
):
    """
    Fetch personalized content for a specific chapter
    """
    try:
        # Get the user's profile if authenticated
        user_profile = None
        if current_user:
            # Fetch user profile from database using async session
            statement = select(UserProfile).where(UserProfile.user_id == current_user.id)
            result = await session.execute(statement)
            # AsyncResult.first() returns a Row; scalars() unwraps to the UserProfile model
            user_profile = result.scalars().first()

        # Get the original content
        # Resolve repo root reliably (Windows/Linux). This file is at: backend/src/api/personalization_router.py
        repo_root = Path(__file__).resolve().parents[3]
        docs_path = repo_root / "frontend" / "docs"
        content_service = ContentService(content_base_path=str(docs_path))
        original_content = await content_service.get_content_by_path(chapter_path)

        # Apply personalization based on user profile
        personalization_service = PersonalizationService()
        personalized_content = await personalization_service.personalize_content(
            content=original_content,
            user_profile=user_profile
        )

        response_data = {
            "chapter_path": chapter_path,
            "personalized_content": personalized_content,
            "user_profile": {
                "software_background": getattr(user_profile, 'software_background', []),
                "hardware_background": getattr(user_profile, 'hardware_background', []),
                "experience_level": getattr(user_profile, 'experience_level', 'beginner')
            } if user_profile else None,
            "personalization_applied": user_profile is not None
        }

        return response_data
    except HTTPException:
        # Re-raise HTTP exceptions (like 404 for content not found)
        raise
    except Exception as e:
        logger.error(f"Error getting personalized content for {chapter_path}: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error getting personalized content: {str(e)}"
        )


@router.post("/preview", response_model=PreviewPersonalizationResponse)
async def preview_personalized_content(
    request: PreviewPersonalizationRequest,
    current_user: Optional[User] = Depends(get_current_user),
    session: AsyncSession = Depends(get_session)
):
    """
    Preview personalized content with custom profile data
    """
    try:
        chapter_path = request.chapter_path
        profile_data = request.profile_data

        if not chapter_path:
            raise HTTPException(
                status_code=400,
                detail="chapter_path is required"
            )

        # Get the original content
        # Resolve repo root reliably (Windows/Linux). This file is at: backend/src/api/personalization_router.py
        repo_root = Path(__file__).resolve().parents[3]
        docs_path = repo_root / "frontend" / "docs"
        content_service = ContentService(content_base_path=str(docs_path))
        original_content = await content_service.get_content_by_path(chapter_path)

        # Apply personalization based on provided profile data
        personalization_service = PersonalizationService()
        personalized_content = await personalization_service.personalize_content(
            content=original_content,
            user_profile=profile_data
        )

        response_data = {
            "chapter_path": chapter_path,
            "personalized_content": personalized_content,
            "applied_profile": profile_data,
            "personalization_applied": True
        }

        return response_data
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error previewing personalized content: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error previewing personalized content: {str(e)}"
        )


class PersonalizationRuleResponse(BaseModel):
    id: int
    content_path: str
    description: str
    user_profile_criteria: Dict[str, Any]
    priority: int
    is_active: bool
    created_at: Optional[str]
    updated_at: Optional[str]

class PersonalizationRulesResponse(BaseModel):
    rules: list[PersonalizationRuleResponse]

@router.get("/rules", response_model=PersonalizationRulesResponse)
async def get_personalization_rules(
    current_user: User = Depends(get_current_user),
    session: AsyncSession = Depends(get_session)
):
    """
    Get list of all active personalization rules
    """
    try:
        # Check if user has admin privileges to access rules
        # For now, allowing access to all authenticated users
        statement = select(PersonalizationRule).where(PersonalizationRule.is_active == True)
        result = await session.execute(statement)
        rules = result.all()

        # Convert rules to response format
        rules_response = []
        for rule in rules:
            rules_response.append(PersonalizationRuleResponse(
                id=rule.id,
                content_path=rule.content_path,
                description=rule.description,
                user_profile_criteria=rule.user_profile_criteria,
                priority=rule.priority,
                is_active=rule.is_active,
                created_at=rule.created_at.isoformat() if rule.created_at else None,
                updated_at=rule.updated_at.isoformat() if rule.updated_at else None,
            ))

        return PersonalizationRulesResponse(rules=rules_response)
    except Exception as e:
        logger.error(f"Error getting personalization rules: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error getting personalization rules: {str(e)}"
        )


class PersonalizationRuleUpdateRequest(BaseModel):
    content_path: Optional[str] = None
    user_profile_criteria: Optional[Dict[str, Any]] = None
    content_variants: Optional[Dict[str, Any]] = None
    description: Optional[str] = None
    priority: Optional[int] = None
    is_active: Optional[bool] = None

@router.put("/rules/{rule_id}", response_model=PersonalizationRuleResponse)
async def update_personalization_rule(
    rule_id: int,
    request: PersonalizationRuleUpdateRequest,
    current_user: User = Depends(get_current_user),
    session: AsyncSession = Depends(get_session)
):
    """
    Update a personalization rule
    """
    try:
        # Find the rule by ID
        statement = select(PersonalizationRule).where(PersonalizationRule.id == rule_id)
        result = await session.execute(statement)
        rule = result.first()

        if not rule:
            raise HTTPException(
                status_code=404,
                detail=f"Personalization rule with ID {rule_id} not found"
            )

        # Update the rule with the provided data
        update_data = {
            "content_path": request.content_path,
            "user_profile_criteria": request.user_profile_criteria,
            "description": request.description if request.description is not None else rule.description,
            "priority": request.priority if request.priority is not None else rule.priority,
            "is_active": request.is_active if request.is_active is not None else rule.is_active,
        }

        # Update the fields
        for field, value in update_data.items():
            if value is not None:
                setattr(rule, field, value)

        # Update the timestamp
        from datetime import datetime
        rule.updated_at = datetime.utcnow()

        # Commit the changes
        session.add(rule)
        await session.commit()
        await session.refresh(rule)

        # Return the updated rule
        return PersonalizationRuleResponse(
            id=rule.id,
            content_path=rule.content_path,
            description=rule.description,
            user_profile_criteria=rule.user_profile_criteria,
            priority=rule.priority,
            is_active=rule.is_active,
            created_at=rule.created_at.isoformat() if rule.created_at else None,
            updated_at=rule.updated_at.isoformat() if rule.updated_at else None,
        )
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error updating personalization rule {rule_id}: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error updating personalization rule: {str(e)}"
        )