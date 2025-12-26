from typing import Optional, List, Dict
from sqlmodel import SQLModel
from datetime import datetime

class UserBase(SQLModel):
    username: str
    email: str
    software_background: Optional[List[str]] = None
    hardware_background: Optional[List[str]] = None

class UserCreate(UserBase):
    password: str

class UserRead(UserBase):
    id: int
    oauth_provider_ids: Optional[Dict[str, str]] = None

class UserCreateOAuth(UserBase):
    oauth_provider_ids: Dict[str, str] # Requires provider and ID

class Token(SQLModel):
    access_token: str
    token_type: str

class TokenData(SQLModel):
    user_id: Optional[int] = None

class PersonalizationDataRead(SQLModel):
    software_background: Optional[List[str]] = None
    hardware_background: Optional[List[str]] = None

class UserLogin(SQLModel):
    email: str
    password: str