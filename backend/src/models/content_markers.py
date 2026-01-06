<<<<<<< HEAD
from datetime import datetime
from typing import Optional
from sqlmodel import SQLModel, Field


class ContentMarkerBase(SQLModel):
    """Base model for personalization marker without ID"""
    content_path: str = Field(index=True)
    marker_type: str = Field(max_length=100)
    marker_location: Optional[str] = Field(default=None)
    description: Optional[str] = Field(default=None)


class ContentMarker(ContentMarkerBase, table=True):
    """Content Marker model for registering personalization markers found in content"""
    __tablename__ = "content_markers"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)


class ContentMarkerCreate(ContentMarkerBase):
    """Model for creating a new content marker"""
    pass


class ContentMarkerRead(ContentMarkerBase):
    """Model for reading content marker data"""
    id: int
=======
from datetime import datetime
from typing import Optional
from sqlmodel import SQLModel, Field


class ContentMarkerBase(SQLModel):
    """Base model for personalization marker without ID"""
    content_path: str = Field(index=True)
    marker_type: str = Field(max_length=100)
    marker_location: Optional[str] = Field(default=None)
    description: Optional[str] = Field(default=None)


class ContentMarker(ContentMarkerBase, table=True):
    """Content Marker model for registering personalization markers found in content"""
    __tablename__ = "content_markers"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)


class ContentMarkerCreate(ContentMarkerBase):
    """Model for creating a new content marker"""
    pass


class ContentMarkerRead(ContentMarkerBase):
    """Model for reading content marker data"""
    id: int
>>>>>>> 012-docusaurus-i18n-urdu
    created_at: datetime