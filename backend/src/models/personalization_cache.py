<<<<<<< HEAD
from datetime import datetime
from typing import Optional
from sqlmodel import SQLModel, Field


class PersonalizationCacheBase(SQLModel):
    """Base model for personalization cache without ID"""
    user_profile_hash: str = Field(index=True)
    content_path: str = Field(index=True)
    personalized_content: str
    cache_expires_at: datetime


class PersonalizationCache(PersonalizationCacheBase, table=True):
    """Personalization Cache model for storing pre-computed personalized content"""
    __tablename__ = "personalization_cache"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    cache_created_at: datetime = Field(default_factory=datetime.utcnow)


class PersonalizationCacheCreate(PersonalizationCacheBase):
    """Model for creating a new personalization cache entry"""
    pass


class PersonalizationCacheRead(PersonalizationCacheBase):
    """Model for reading personalization cache data"""
    id: int
=======
from datetime import datetime
from typing import Optional
from sqlmodel import SQLModel, Field


class PersonalizationCacheBase(SQLModel):
    """Base model for personalization cache without ID"""
    user_profile_hash: str = Field(index=True)
    content_path: str = Field(index=True)
    personalized_content: str
    cache_expires_at: datetime


class PersonalizationCache(PersonalizationCacheBase, table=True):
    """Personalization Cache model for storing pre-computed personalized content"""
    __tablename__ = "personalization_cache"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    cache_created_at: datetime = Field(default_factory=datetime.utcnow)


class PersonalizationCacheCreate(PersonalizationCacheBase):
    """Model for creating a new personalization cache entry"""
    pass


class PersonalizationCacheRead(PersonalizationCacheBase):
    """Model for reading personalization cache data"""
    id: int
>>>>>>> 012-docusaurus-i18n-urdu
    cache_created_at: datetime