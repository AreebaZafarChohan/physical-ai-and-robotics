import os
import time # Added import
from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
from uuid import UUID
from openai import OpenAI
from backend.app.services.llm_safety import sanitize_llm_response
from backend.app.metrics import record_query_response_time # Added import

router = APIRouter()

class SelectedTextRequest(BaseModel):
    query: str
    selected_text: str
    user_id: Optional[UUID] = None
    language: str # "en", "ur", "ar"

class SelectedTextResponse(BaseModel):
    answer: str

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
if not OPENAI_API_KEY:
    raise ValueError("OPENAI_API_KEY environment variable is not set.")

openai_client = OpenAI(api_key=OPENAI_API_KEY)

@router.post("/selected_text", response_model=SelectedTextResponse)
async def ask_from_selected_text(request: SelectedTextRequest):
    start_time = time.perf_counter() # Start timing
    try:
        if request.language not in ["en", "ur", "ar"]:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Invalid language. Must be 'en', 'ur', or 'ar'.")

        # Construct the prompt for the LLM using ONLY the selected text as context
        system_prompt = (
            f"You are a helpful assistant. Answer the user's question concisely based ONLY on the provided selected text. "
            f"If the answer is not in the selected text, state that you don't know based on the provided information. "
            f"Maintain the conversation in {request.language}."
        )

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": f"Selected Text: {request.selected_text}\n\nQuestion: {request.query}"}
        ]

        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo", # Or a more advanced model like "gpt-4"
            messages=messages,
            temperature=0.7,
            max_tokens=500,
        )
        answer = response.choices[0].message.content.strip()
        
        # Sanitize LLM response
        sanitized_answer = sanitize_llm_response(answer)

        final_response = SelectedTextResponse(answer=sanitized_answer)

        end_time = time.perf_counter() # End timing
        record_query_response_time(endpoint="/selected_text", duration_ms=(end_time - start_time) * 1000, status_code=status.HTTP_200_OK)
        return final_response

    except HTTPException as e:
        end_time = time.perf_counter() # End timing for error
        record_query_response_time(endpoint="/selected_text", duration_ms=(end_time - start_time) * 1000, status_code=e.status_code)
        raise e
    except Exception as e:
        end_time = time.perf_counter() # End timing for error
        record_query_response_time(endpoint="/selected_text", duration_ms=(end_time - start_time) * 1000, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=f"Error generating response from LLM: {e}")