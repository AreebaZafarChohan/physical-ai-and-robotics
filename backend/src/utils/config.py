import os
from functools import lru_cache
from dotenv import load_dotenv

load_dotenv()
@lru_cache()
def get_settings():
    class Settings:
        GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY")
        GEMINI_API_BASE_URL: str = os.getenv("GEMINI_API_BASE_URL")
        QDRANT_URL: str = os.getenv("QDRANT_URL")
        QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY")
        QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME")
        COHERE_API_KEY: str = os.getenv("COHERE_API_KEY")

        # Validate that essential environment variables are set
        def __post_init__(self):
            if not self.GEMINI_API_KEY:
                raise ValueError("GEMINI_API_KEY environment variable not set.")
            if not self.GEMINI_API_BASE_URL:
                raise ValueError("GEMINI_API_BASE_URL environment variable not set.")
            if not self.QDRANT_URL:
                raise ValueError("QDRANT_URL environment variable not set.")
            if not self.QDRANT_COLLECTION_NAME:
                raise ValueError("QDRANT_COLLECTION_NAME environment variable not set.")
            if not self.COHERE_API_KEY:
                raise ValueError("COHERE_API_KEY environment variable not set.")

    return Settings()

if __name__ == '__main__':
    # For testing purposes, you might need to set dummy env vars
    # In a real scenario, these would be loaded from a .env file or system env
    os.environ["GEMINI_API_KEY"] = "test_gemini_key"
    os.environ["GEMINI_API_BASE_URL"] = "http://localhost:8080/v1"
    os.environ["QDRANT_URL"] = "http://localhost:6333"
    os.environ["QDRANT_COLLECTION_NAME"] = "test_collection"
    os.environ["COHERE_API_KEY"] = "test_cohere_key"

    try:
        settings = get_settings()
        print("Settings loaded successfully:")
        print(f"GEMINI_API_KEY: {settings.GEMINI_API_KEY}")
        print(f"QDRANT_URL: {settings.QDRANT_URL}")
    except ValueError as e:
        print(f"Error loading settings: {e}")