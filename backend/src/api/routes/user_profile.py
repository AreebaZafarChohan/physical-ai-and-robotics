from typing import Optional
from fastapi import APIRouter, Depends, HTTPException, status
from sqlmodel import Session, select

from backend.src.database import get_session
from backend.src.models.user_profile import UserProfile, UserProfileCreate, UserProfileRead
from backend.src.services.user_service import get_current_user
from backend.src.models.user import User

router = APIRouter(prefix="/user-profile", tags=["user-profile"])


@router.get("/", response_model=UserProfileRead)
async def get_user_profile(
    current_user: User = Depends(get_current_user),
    session: Session = Depends(get_session)
):
    """
    Get the current user's profile information
    """
    statement = select(UserProfile).where(UserProfile.user_id == current_user.id)
    user_profile = session.exec(statement).first()
    
    if not user_profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User profile not found"
        )
    
    return user_profile


@router.post("/", response_model=UserProfileRead)
async def create_user_profile(
    profile_data: UserProfileCreate,
    current_user: User = Depends(get_current_user),
    session: Session = Depends(get_session)
):
    """
    Create a new user profile
    """
    # Check if user already has a profile
    statement = select(UserProfile).where(UserProfile.user_id == current_user.id)
    existing_profile = session.exec(statement).first()
    
    if existing_profile:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="User profile already exists. Use PUT to update."
        )
    
    # Create new profile
    user_profile = UserProfile(
        user_id=current_user.id,
        software_background=profile_data.software_background,
        hardware_background=profile_data.hardware_background,
        experience_level=profile_data.experience_level
    )
    
    session.add(user_profile)
    session.commit()
    session.refresh(user_profile)
    
    return user_profile


@router.put("/", response_model=UserProfileRead)
async def update_user_profile(
    profile_data: UserProfileCreate,  # Using Create model since it has all necessary fields
    current_user: User = Depends(get_current_user),
    session: Session = Depends(get_session)
):
    """
    Update the current user's profile information
    """
    statement = select(UserProfile).where(UserProfile.user_id == current_user.id)
    user_profile = session.exec(statement).first()
    
    if not user_profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User profile not found. Create a profile first."
        )
    
    # Update profile
    user_profile.software_background = profile_data.software_background
    user_profile.hardware_background = profile_data.hardware_background
    user_profile.experience_level = profile_data.experience_level
    
    session.add(user_profile)
    session.commit()
    session.refresh(user_profile)
    
    return user_profile