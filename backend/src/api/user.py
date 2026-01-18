from typing import Annotated, Optional, List
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from sqlalchemy.ext.asyncio import AsyncSession
from sqlmodel import select
from pydantic import BaseModel

from backend.src.database import get_session
from backend.src.models.user import User
from backend.src.models.user_profile import UserProfile
from backend.src.utils.auth import verify_access_token
from backend.src.utils.errors import CredentialException
from backend.src.schemas import UserRead, PersonalizationDataRead

router = APIRouter()


# Request model for profile update
class ProfileUpdateRequest(BaseModel):
    software_background: Optional[List[str]] = None
    hardware_background: Optional[List[str]] = None
    experience_level: Optional[str] = None


# Dependency to get current user
async def get_current_user(
    token: Annotated[str, Depends(OAuth2PasswordBearer(tokenUrl="auth/login"))],
    session: Annotated[AsyncSession, Depends(get_session)]
) -> User:
    credentials_exception = CredentialException()
    user_id = verify_access_token(token, credentials_exception)
    user = await session.get(User, int(user_id))
    if user is None:
        raise credentials_exception
    return user


@router.get("/profile", response_model=UserRead)
async def read_users_me(
    current_user: Annotated[User, Depends(get_current_user)]
):
    return current_user


@router.put("/profile/update")
async def update_user_profile(
    profile_data: ProfileUpdateRequest,
    current_user: Annotated[User, Depends(get_current_user)],
    session: Annotated[AsyncSession, Depends(get_session)]
):
    """
    Update user profile with software/hardware background and experience level.
    Creates a new profile if one doesn't exist.
    """
    try:
        # Find existing profile or create new one
        statement = select(UserProfile).where(UserProfile.user_id == current_user.id)
        existing_profile = (await session.execute(statement)).first()

        if existing_profile:
            # Update existing profile
            if profile_data.software_background is not None:
                existing_profile.software_background = profile_data.software_background
            if profile_data.hardware_background is not None:
                existing_profile.hardware_background = profile_data.hardware_background
            if profile_data.experience_level is not None:
                existing_profile.experience_level = profile_data.experience_level

            from datetime import datetime
            existing_profile.updated_at = datetime.utcnow()

            session.add(existing_profile)
            await session.commit()
            await session.refresh(existing_profile)

            return {
                "message": "Profile updated successfully",
                "profile": {
                    "user_id": existing_profile.user_id,
                    "software_background": existing_profile.software_background,
                    "hardware_background": existing_profile.hardware_background,
                    "experience_level": existing_profile.experience_level,
                    "updated_at": existing_profile.updated_at
                }
            }
        else:
            # Create new profile
            new_profile = UserProfile(
                user_id=current_user.id,
                software_background=profile_data.software_background or [],
                hardware_background=profile_data.hardware_background or [],
                experience_level=profile_data.experience_level or "beginner"
            )
            session.add(new_profile)
            await session.commit()
            await session.refresh(new_profile)

            return {
                "message": "Profile created successfully",
                "profile": {
                    "user_id": new_profile.user_id,
                    "software_background": new_profile.software_background,
                    "hardware_background": new_profile.hardware_background,
                    "experience_level": new_profile.experience_level,
                    "created_at": new_profile.created_at
                }
            }

    except Exception as e:
        await session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to update profile: {str(e)}"
        )


@router.get("/profile/details")
async def get_user_profile_details(
    current_user: Annotated[User, Depends(get_current_user)],
    session: Annotated[AsyncSession, Depends(get_session)]
):
    """
    Get detailed user profile including software/hardware background.
    """
    statement = select(UserProfile).where(UserProfile.user_id == current_user.id)
    profile = (await session.execute(statement)).first()

    return {
        "user": {
            "id": current_user.id,
            "username": current_user.username,
            "email": current_user.email,
        },
        "profile": {
            "software_background": profile.software_background if profile else [],
            "hardware_background": profile.hardware_background if profile else [],
            "experience_level": profile.experience_level if profile else "beginner",
        } if profile else None
    }


@router.get("/personalization-data", response_model=PersonalizationDataRead)
async def read_personalization_data(
    current_user: Annotated[User, Depends(get_current_user)]
):
    if not current_user.personalization_data:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Personalization data not found for this user."
        )
    return current_user.personalization_data