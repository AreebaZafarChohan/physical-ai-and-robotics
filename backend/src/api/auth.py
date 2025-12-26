from datetime import timedelta
from typing import Annotated
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from sqlmodel import Session
from jose import JWTError

from backend.src.database import get_session
from backend.src.models.user import User
from backend.src.services.user_service import UserService
from backend.src.utils.auth import create_access_token, verify_access_token
from backend.src.utils.errors import CredentialException, UserAlreadyExistsException, InvalidCredentialsException
from backend.src.schemas import Token, UserCreate, UserRead, UserCreateOAuth, UserLogin

router = APIRouter()

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

ACCESS_TOKEN_EXPIRE_MINUTES = 30

async def get_current_user(
    token: Annotated[str, Depends(oauth2_scheme)],
    session: Annotated[Session, Depends(get_session)]
) -> User:
    credentials_exception = CredentialException()
    try:
        user_id_str = verify_access_token(token, credentials_exception)
        if user_id_str is None:
            raise credentials_exception
        
        try:
            user_id = int(user_id_str)
        except (ValueError, TypeError):
            raise credentials_exception

        user_service = UserService(session)
        user = user_service.get_user_by_id(user_id)
        if user is None:
            raise credentials_exception
        return user
    except JWTError:
        raise credentials_exception

@router.post("/signup", response_model=Token)
def signup_user(
    user_create: UserCreate,
    session: Annotated[Session, Depends(get_session)]
):
    user_service = UserService(session)
    try:
        user = user_service.create_user_with_password(
            username=user_create.username,
            email=user_create.email,
            password=user_create.password,
            software_background=user_create.software_background,
            hardware_background=user_create.hardware_background
        )
        access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        access_token = create_access_token(
            data={"sub": str(user.id)}, expires_delta=access_token_expires
        )
        return {"access_token": access_token, "token_type": "bearer"}
    except UserAlreadyExistsException as e:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=e.detail
        )

@router.post("/login", response_model=Token)
async def login_for_access_token(
    user_login: UserLogin,
    session: Annotated[Session, Depends(get_session)]
):
    user_service = UserService(session)
    user = user_service.authenticate_user(user_login.email, user_login.password)
    if not user:
        raise InvalidCredentialsException()
    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": str(user.id)}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}

@router.post("/logout")
async def logout(current_user: Annotated[User, Depends(get_current_user)]):
    # In a real application, you might invalidate the token or add it to a blacklist.
    # For stateless JWTs, logout is primarily a client-side operation (deleting the token).
    return {"message": "Successfully logged out"}

@router.post("/signup/oauth", response_model=Token)
def signup_oauth(
    user_create_oauth: UserCreateOAuth,
    session: Annotated[Session, Depends(get_session)]
):
    user_service = UserService(session)
    try:
        user = user_service.create_user_with_oauth(
            username=user_create_oauth.username,
            email=user_create_oauth.email,
            oauth_provider_ids=user_create_oauth.oauth_provider_ids,
            software_background=user_create_oauth.software_background,
            hardware_background=user_create_oauth.hardware_background
        )
        access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        access_token = create_access_token(
            data={"sub": str(user.id)}, expires_delta=access_token_expires
        )
        return {"access_token": access_token, "token_type": "bearer"}
    except UserAlreadyExistsException as e:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=e.detail
        )