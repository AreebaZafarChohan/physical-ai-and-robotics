import os
import time # Added import
from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
from uuid import UUID
from openai import OpenAI
import re
from backend.app.services.llm_safety import sanitize_llm_response
from backend.app.metrics import record_query_response_time # Added import

router = APIRouter()

class Message(BaseModel):
    role: str
    content: str

class AskRequest(BaseModel):
    query: str
    context: List[str]
    chat_history: List[Message]
    user_id: Optional[UUID] = None
    language: str
    mode: str

class AskResponse(BaseModel):
    answer: str
    citation_links: List[str]

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
if not OPENAI_API_KEY:
    raise ValueError("OPENAI_API_KEY environment variable is not set.")

openai_client = OpenAI(api_key=OPENAI_API_KEY)

@router.post("/ask", response_model=AskResponse)
async def ask_llm(request: AskRequest):
    start_time = time.perf_counter() # Start timing
    try:
        if request.mode not in ["book", "course"]:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Invalid mode. Must be 'book' or 'course'.")
        
        if request.language not in ["en", "ur", "ar"]:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Invalid language. Must be 'en', 'ur', or 'ar'.")

        assistant_role = ""
        if request.mode == "book":
            assistant_role = "an assistant for a textbook on Physical AI & Humanoid Robotics"
        elif request.mode == "course":
            assistant_role = "an assistant for a course on Physical AI & Humanoid Robotics, drawing from textbook and supplementary materials"

        system_prompt = (
            f"You are a helpful {assistant_role}. "
            f"Answer the user's question concisely based ONLY on the provided context. "
            f"If the answer is not in the context, state that you don't know. "
            f"Maintain the conversation in {request.language}. "
            f"Cite your sources using the source links provided in the context."
        )

        messages = [{"role": "system", "content": system_prompt}]

        for msg in request.chat_history:
            messages.append({"role": msg.role, "content": msg.content})

        context_str = "\n".join(request.context)
        messages.append({"role": "user", "content": f"Context: {context_str}\n\nQuestion: {request.query}"})

        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=messages,
            temperature=0.7,
            max_tokens=500,
            # stream=True # For streaming responses, would require a different return type (StreamingResponse)
        )
        answer = response.choices[0].message.content.strip()

        # Sanitize LLM response
        sanitized_answer = sanitize_llm_response(answer) # Added safety mechanism

        citation_links = []
        for ctx_item in request.context:
            urls = re.findall(r'https?://[^\s<>" ]+|www\.[^\s<>" ]+', ctx_item)
            for url in urls:
                if url not in citation_links:
                    citation_links.append(url)
        
        final_response = AskResponse(answer=sanitized_answer, citation_links=citation_links)

        end_time = time.perf_counter() # End timing
        record_query_response_time(endpoint="/ask", duration_ms=(end_time - start_time) * 1000, status_code=status.HTTP_200_OK)
        return final_response

    except HTTPException as e:
        end_time = time.perf_counter() # End timing for error
        record_query_response_time(endpoint="/ask", duration_ms=(end_time - start_time) * 1000, status_code=e.status_code)
        raise e
    except Exception as e:
        end_time = time.perf_counter() # End timing for error
        record_query_response_time(endpoint="/ask", duration_ms=(end_time - start_time) * 1000, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=f"Error generating response from LLM: {e}")