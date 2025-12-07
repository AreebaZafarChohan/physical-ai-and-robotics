from datetime import datetime
from typing import Optional, List
from uuid import UUID, uuid4
from sqlmodel import Field, SQLModel, Relationship


class User(SQLModel, table=True):
    id: Optional[UUID] = Field(default_factory=uuid4, primary_key=True)
    email: Optional[str] = Field(default=None, unique=True, index=True)
    preferences: dict = Field(default_factory=dict)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    last_login_at: Optional[datetime] = None

    sessions: List["ChatSession"] = Relationship(back_populates="user")
    feedback: List["Feedback"] = Relationship(back_populates="user")


class ChatSession(SQLModel, table=True):
    id: Optional[UUID] = Field(default_factory=uuid4, primary_key=True)
    user_id: Optional[UUID] = Field(default=None, foreign_key="user.id")
    started_at: datetime = Field(default_factory=datetime.utcnow)
    last_activity_at: datetime = Field(default_factory=datetime.utcnow)
    mode: str
    status: str

    user: Optional[User] = Relationship(back_populates="sessions")
    messages: List["Message"] = Relationship(back_populates="session")


class Message(SQLModel, table=True):
    id: Optional[UUID] = Field(default_factory=uuid4, primary_key=True)
    session_id: UUID = Field(foreign_key="chatsession.id")
    sender: str
    content: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    citations: List[dict] = Field(default_factory=list)
    language: str

    session: ChatSession = Relationship(back_populates="messages")
    feedback: Optional["Feedback"] = Relationship(back_populates="message")


class Feedback(SQLModel, table=True):
    id: Optional[UUID] = Field(default_factory=uuid4, primary_key=True)
    message_id: Optional[UUID] = Field(default=None, foreign_key="message.id")
    user_id: Optional[UUID] = Field(default=None, foreign_key="user.id")
    type: str
    comment: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)

    message: Optional[Message] = Relationship(back_populates="feedback")
    user: Optional[User] = Relationship(back_populates="feedback")


class BookMetadata(SQLModel, table=True):
    id: Optional[UUID] = Field(default_factory=uuid4, primary_key=True)
    chapter: str
    source_link: str
    paragraph_id: str = Field(index=True)
    text_content_hash: str
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
