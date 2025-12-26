from typing import Optional
from fastapi import Depends, HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlmodel import Session, select
from backend.src.database import get_session
from backend.src.models.user import User
from backend.src.utils.auth import verify_access_token
from backend.src.utils.security import verify_password, get_password_hash
from backend.src.utils.errors import UserAlreadyExistsException
from typing import Optional
from fastapi import Depends
from backend.src.models.user import User

security = HTTPBearer(auto_error=False)

async def get_current_user_optional(
    session: Session = Depends(get_session),
    token: Optional[HTTPAuthorizationCredentials] = Depends(security)
) -> Optional[User]:
    """
    Get current user if authenticated, otherwise return None.
    This is used for endpoints that work for both authenticated and unauthenticated users.
    """
    if token is None:
        return None

    try:
        credentials_exception = HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

        token_data = verify_access_token(token.credentials, credentials_exception)
        if token_data is None:
            return None

        # Convert user_id from string to integer
        try:
            user_id = int(token_data)
        except (ValueError, TypeError):
            return None

        statement = select(User).where(User.id == user_id)
        user = session.exec(statement).first()

        return user
    except Exception:
        return None

async def get_current_user(
    session: Session = Depends(get_session),
    token: HTTPAuthorizationCredentials = Depends(HTTPBearer())
) -> User:
    """
    Dependency to get the current authenticated user from JWT token.
    Raises 401 if token is missing or invalid.
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    token_data = verify_access_token(token.credentials, credentials_exception)
    if token_data is None:
        raise credentials_exception

    # Convert user_id from string to integer
    try:
        user_id = int(token_data)
    except (ValueError, TypeError):
        raise credentials_exception

    statement = select(User).where(User.id == user_id)
    user = session.exec(statement).first()

    if user is None:
        raise credentials_exception

    return user


class UserService:
    """
    Service class for user-related operations
    """
    
    def __init__(self, session: Session):
        self.session = session
    
    def get_user_by_id(self, user_id: int) -> Optional[User]:
        """
        Get a user by their ID
        """
        statement = select(User).where(User.id == user_id)
        return self.session.exec(statement).first()

    def get_user_by_email(self, email: str) -> Optional[User]:
        """
        Get a user by their email address
        """
        statement = select(User).where(User.email == email)
        return self.session.exec(statement).first()

    def authenticate_user(self, email: str, password: str) -> Optional[User]:
        """
        Authenticate a user by email and password
        """
        user = self.get_user_by_email(email)
        if not user or not user.password_hash:
            return None
        if not verify_password(password, user.password_hash):
            return None
        return user

    def create_user_with_password(self, username: str, email: str, password: str, software_background: Optional[list[str]] = None, hardware_background: Optional[list[str]] = None) -> User:
        """
        Create a new user with a password
        """
        if self.get_user_by_email(email):
            raise UserAlreadyExistsException()
        
        hashed_password = get_password_hash(password)
        user = User(
            username=username,
            email=email,
            password_hash=hashed_password,
        )
        self.session.add(user)
        self.session.commit()
        self.session.refresh(user)
        return user

    def create_user_with_oauth(self, username: str, email: str, oauth_provider_ids: dict, software_background: Optional[list[str]] = None, hardware_background: Optional[list[str]] = None) -> User:
        """
        Create a new user with OAuth
        """
        if self.get_user_by_email(email):
            raise UserAlreadyExistsException()

        user = User(
            username=username,
            email=email,
            oauth_provider_ids=oauth_provider_ids,
        )
        self.session.add(user)
        self.session.commit()
        self.session.refresh(user)
        return user

    async def get_user_profile(self, user_id: int) -> Optional[dict]:
        """
        Get user profile information with caching
        """
        # First, try to get from cache
        from backend.src.cache import get_cached_user_profile, cache_user_profile
        cached_profile = await get_cached_user_profile(user_id)
        if cached_profile:
            return cached_profile

        # If not in cache, get from database
        from backend.src.models.user_profile import UserProfile

        statement = select(UserProfile).where(UserProfile.user_id == user_id)
        user_profile = self.session.exec(statement).first()

        if user_profile:
            profile_data = {
                "id": user_profile.id,
                "user_id": user_profile.user_id,
                "software_background": user_profile.software_background,
                "hardware_background": user_profile.hardware_background,
                "experience_level": user_profile.experience_level,
                "created_at": user_profile.created_at,
                "updated_at": user_profile.updated_at
            }

            # Cache the profile data
            await cache_user_profile(user_id, profile_data)

            return profile_data

        return None

    def validate_user_profile_for_personalization(self, user_profile: Optional[dict]) -> tuple[bool, str]:
        """
        Validate if a user profile has sufficient information for personalization
        """
        if not user_profile:
            return False, "User profile not found"

        # Check if required fields are present
        software_bg = user_profile.get("software_background", [])
        hardware_bg = user_profile.get("hardware_background", [])

        # A profile is valid if it has either software or hardware background, or experience level
        if not software_bg and not hardware_bg:
            return False, "User profile must contain software or hardware background information"

        # Validate experience level if present
        experience_level = user_profile.get("experience_level", "").lower()
        if experience_level and experience_level not in ["beginner", "intermediate", "advanced"]:
            return False, f"Invalid experience level: {experience_level}. Must be one of: beginner, intermediate, advanced"

        return True, "Profile is valid for personalization"
    
    def update_user_profile(self, user_id: int, profile_data: dict) -> Optional[dict]:
        """
        Update user profile information
        """
        from backend.src.models.user_profile import UserProfile
        
        statement = select(UserProfile).where(UserProfile.user_id == user_id)
        user_profile = self.session.exec(statement).first()
        
        if not user_profile:
            # Create new profile if it doesn't exist
            user_profile = UserProfile(
                user_id=user_id,
                software_background=profile_data.get("software_background", []),
                hardware_background=profile_data.get("hardware_background", []),
                experience_level=profile_data.get("experience_level", "beginner")
            )
            self.session.add(user_profile)
        else:
            # Update existing profile
            user_profile.software_background = profile_data.get("software_background", user_profile.software_background)
            user_profile.hardware_background = profile_data.get("hardware_background", user_profile.hardware_background)
            user_profile.experience_level = profile_data.get("experience_level", user_profile.experience_level)
            self.session.add(user_profile)
        
        self.session.commit()
        
        return {
            "software_background": user_profile.software_background,
            "hardware_background": user_profile.hardware_background,
            "experience_level": user_profile.experience_level
        }
