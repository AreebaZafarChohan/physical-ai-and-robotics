from datetime import datetime
from typing import Dict, Optional
from sqlmodel import SQLModel, Field, Column, JSON
from pydantic import BaseModel


class PersonalizationRuleBase(SQLModel):
    """Base model for personalization rule without ID"""
    content_path: str = Field(index=True)
    user_profile_criteria: Dict = Field(sa_column=Column(JSON))
    content_variants: Dict = Field(sa_column=Column(JSON))
    description: Optional[str] = Field(default=None)
    priority: int = Field(default=10)
    is_active: bool = Field(default=True)


class PersonalizationRule(PersonalizationRuleBase, table=True):
    """Personalization Rule model for storing content personalization rules"""
    __tablename__ = "personalization_rules"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)


class PersonalizationRuleCreate(PersonalizationRuleBase):
    """Model for creating a new personalization rule"""
    pass


class PersonalizationRuleRead(PersonalizationRuleBase):
    """Model for reading personalization rule data"""
    id: int
    created_at: datetime
    updated_at: datetime


class PersonalizationRuleUpdate(PersonalizationRuleBase):
    """Model for updating a personalization rule"""
    pass