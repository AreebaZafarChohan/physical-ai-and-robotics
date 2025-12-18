import os
from dotenv import load_dotenv
from agents import OpenAIChatCompletionsModel, AsyncOpenAI

load_dotenv()

class GeminiModel:
    def __init__(self):
        self.api_key = os.getenv("GEMINI_API_KEY")
        self.base_url = os.getenv("GEMINI_API_BASE_URL")

        if not self.api_key or not self.base_url:
            raise ValueError(
                "GEMINI_API_KEY and GEMINI_API_BASE_URL must be set"
            )

        self.client = AsyncOpenAI(
            api_key=self.api_key,
            base_url=self.base_url,
        )

        self.chat_model = OpenAIChatCompletionsModel(
            model="gemini-2.5-flash",
            openai_client=self.client,
        )


# Example usage (for testing purposes, will be removed or refactored later)
if __name__ == "__main__":
    # In a real application, environment variables would be loaded by a config utility
    os.environ["GEMINI_API_KEY"] = "sk-your-test-key"
    os.environ["GEMINI_API_BASE_URL"] = "http://localhost:8080/v1" # Example base URL
    
    try:
        gemini_model = GeminiModel()
        print("GeminiModel client initialized successfully.")
        print(f"Chat model: {gemini_model.chat_model.model_name}")
        # You can now use gemini_model.client for interactions
    except ValueError as e:
        print(f"Error initializing GeminiModel: {e}")