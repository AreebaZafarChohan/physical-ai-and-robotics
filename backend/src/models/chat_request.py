from pydantic import BaseModel
from typing import Optional

class ChatRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None