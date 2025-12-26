from typing import List
from pydantic import BaseModel, ValidationError, validator

class BackgroundData(BaseModel):
    items: List[str]

    @validator('items', each_item=True)
    def validate_item_content(cls, v):
        if not isinstance(v, str) or not v.strip():
            raise ValueError("Each background item must be a non-empty string.")
        if len(v) > 50: # Example length limit
            raise ValueError("Each background item must be less than 50 characters.")
        return v

def validate_background_list(data: List[str]):
    try:
        BackgroundData(items=data)
        return True
    except ValidationError as e:
        print(f"Validation Error: {e.errors()}")
        return False
