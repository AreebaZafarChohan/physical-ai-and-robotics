from typing import List, Optional
from sqlmodel import Field, SQLModel, Relationship
from sqlalchemy import Column, JSON

class PersonalizationData(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    software_background: List[str] = Field(default_factory=list, sa_column=Column(JSON))
    hardware_background: List[str] = Field(default_factory=list, sa_column=Column(JSON))

    user_id: Optional[int] = Field(default=None, foreign_key="user.id")
    user: Optional["User"] = Relationship(back_populates="personalization_data")