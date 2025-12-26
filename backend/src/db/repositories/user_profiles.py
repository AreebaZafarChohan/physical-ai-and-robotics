from typing import List, Optional
from sqlmodel import Session, select
from backend.src.models.user_profile import UserProfile, UserProfileCreate, UserProfileUpdate


class UserProfileRepository:
    """
    Repository class for user profile database operations
    """
    
    def __init__(self, session: Session):
        self.session = session
    
    def get_profile_by_user_id(self, user_id: int) -> Optional[UserProfile]:
        """
        Get a user profile by the user's ID
        """
        statement = select(UserProfile).where(UserProfile.user_id == user_id)
        return self.session.exec(statement).first()

    def get_profile_complete_info(self, user_id: int) -> Optional[dict]:
        """
        Get complete user profile information with additional details
        """
        profile = self.get_profile_by_user_id(user_id)
        if not profile:
            return None

        return {
            "id": profile.id,
            "user_id": profile.user_id,
            "software_background": profile.software_background,
            "hardware_background": profile.hardware_background,
            "experience_level": profile.experience_level,
            "created_at": profile.created_at,
            "updated_at": profile.updated_at
        }
    
    def get_profile_by_id(self, profile_id: int) -> Optional[UserProfile]:
        """
        Get a user profile by its ID
        """
        return self.session.get(UserProfile, profile_id)
    
    def create_profile(self, profile_data: UserProfileCreate) -> UserProfile:
        """
        Create a new user profile
        """
        profile = UserProfile(
            user_id=profile_data.user_id,
            software_background=profile_data.software_background,
            hardware_background=profile_data.hardware_background,
            experience_level=profile_data.experience_level
        )
        
        self.session.add(profile)
        self.session.commit()
        self.session.refresh(profile)
        return profile
    
    def update_profile(self, user_id: int, profile_data: UserProfileCreate) -> Optional[UserProfile]:
        """
        Update an existing user profile
        """
        profile = self.get_profile_by_user_id(user_id)
        if not profile:
            return None
        
        # Update the profile with new data
        profile.software_background = profile_data.software_background
        profile.hardware_background = profile_data.hardware_background
        profile.experience_level = profile_data.experience_level
        
        self.session.add(profile)
        self.session.commit()
        self.session.refresh(profile)
        return profile
    
    def delete_profile(self, user_id: int) -> bool:
        """
        Delete a user profile
        """
        profile = self.get_profile_by_user_id(user_id)
        if not profile:
            return False
        
        self.session.delete(profile)
        self.session.commit()
        return True
    
    def get_profiles_by_software_background(self, software: str) -> List[UserProfile]:
        """
        Get user profiles that have a specific software in their background
        """
        statement = select(UserProfile).where(
            UserProfile.software_background.op('?')(software)  # This is for JSON/JSONB columns in PostgreSQL
        )
        # For SQLModel with JSON fields, we might need a different approach
        # The above is PostgreSQL-specific JSON operator
        
        # For now, getting all profiles and filtering in Python
        all_profiles = self.session.exec(select(UserProfile)).all()
        matching_profiles = [
            profile for profile in all_profiles
            if software in profile.software_background
        ]
        return matching_profiles
    
    def get_profiles_by_hardware_background(self, hardware: str) -> List[UserProfile]:
        """
        Get user profiles that have a specific hardware in their background
        """
        all_profiles = self.session.exec(select(UserProfile)).all()
        matching_profiles = [
            profile for profile in all_profiles
            if hardware in profile.hardware_background
        ]
        return matching_profiles
    
    def get_profiles_by_experience_level(self, level: str) -> List[UserProfile]:
        """
        Get user profiles with a specific experience level
        """
        statement = select(UserProfile).where(UserProfile.experience_level == level)
        return self.session.exec(statement).all()