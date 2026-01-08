from datetime import datetime
from typing import List, Optional
from sqlmodel import SQLModel, Field, create_engine, Column, JSON
from pydantic import BaseModel


class UserProfileBase(SQLModel):
    """Base model for user profile without ID"""
    user_id: int = Field(unique=True, index=True)
    software_background: List[str] = Field(sa_column=Column(JSON))
    hardware_background: List[str] = Field(sa_column=Column(JSON))
    experience_level: str = Field(default="beginner", max_length=20)  # enum: beginner, intermediate, advanced


class UserProfile(UserProfileBase, table=True):
    """User Profile model for storing user background information"""
    __tablename__ = "user_profiles"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)


class UserProfileCreate(UserProfileBase):
    """Model for creating a new user profile"""
    pass


class UserProfileRead(UserProfileBase):
    """Model for reading user profile data"""
    id: int
    created_at: datetime
    updated_at: datetime


class UserProfileUpdate(UserProfileBase):
    """Model for updating user profile data"""
    pass
