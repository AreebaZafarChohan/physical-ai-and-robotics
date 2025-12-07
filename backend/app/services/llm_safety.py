from typing import List

# A very basic keyword blacklist for demonstration purposes
BLACKLISTED_KEYWORDS = [
    "harmful", "dangerous", "unethical", "illegal", "offensive", "hate speech"
]

def sanitize_llm_response(response_text: str) -> str:
    """
    Performs basic sanitization and filtering of LLM responses.
    This is a placeholder for more comprehensive content moderation.
    """
    lower_response = response_text.lower()
    for keyword in BLACKLISTED_KEYWORDS:
        if keyword in lower_response:
            # Replace or flag the response
            # For this example, we'll replace the entire response with a warning
            return "I cannot provide a response that contains inappropriate content. Please rephrase your query."
            
    # Add more sophisticated filtering here (e.g., sentiment analysis, external moderation API calls)
    
    return response_text

def check_response_for_safety(response_text: str) -> bool:
    """
    Checks if the LLM response is safe based on predefined rules.
    Returns True if safe, False otherwise.
    """
    lower_response = response_text.lower()
    for keyword in BLACKLISTED_KEYWORDS:
        if keyword in lower_response:
            return False
    return True
