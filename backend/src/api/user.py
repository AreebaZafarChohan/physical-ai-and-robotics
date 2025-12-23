from typing import Annotated
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer # New import
from sqlmodel import Session, select

from backend.src.database import get_session
from backend.src.models.user import User
from backend.src.utils.auth import verify_access_token
from backend.src.utils.errors import CredentialException
from backend.src.schemas import UserRead, PersonalizationDataRead

router = APIRouter()

# Dependency to get current user
async def get_current_user(
    token: Annotated[str, Depends(OAuth2PasswordBearer(tokenUrl="auth/login"))], # Corrected tokenUrl
    session: Annotated[Session, Depends(get_session)]
) -> User:
    credentials_exception = CredentialException()
    user_id = verify_access_token(token, credentials_exception)
    user = session.get(User, int(user_id))
    if user is None:
        raise credentials_exception
    return user

@router.get("/profile", response_model=UserRead)
async def read_users_me(
    current_user: Annotated[User, Depends(get_current_user)]
):
    return current_user

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

