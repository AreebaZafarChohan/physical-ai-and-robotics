from typing import Optional
from sqlmodel import Field, SQLModel, Relationship
from sqlalchemy import Column, JSON
from .personalization import PersonalizationData # Import PersonalizationData

class User(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    email: str = Field(unique=True, index=True)
    password_hash: str
    oauth_provider_ids: Optional[dict] = Field(default=None, sa_column=Column(JSON))

    personalization_data: Optional[PersonalizationData] = Relationship(back_populates="user")

