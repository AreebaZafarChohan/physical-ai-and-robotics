import os
import pytest
from unittest.mock import patch

from backend.src.models.gemini_model import GeminiModel
from agents import OpenAIChatCompletionsModel
from agents import AsyncOpenAI

def test_gemini_model_initialization_success():
    """
    Tests successful initialization of GeminiModel with valid environment variables.
    """
    with patch.dict(os.environ, {
        "GEMINI_API_KEY": "test_key",
        "GEMINI_API_BASE_URL": "http://test_url.com/v1"
    }):
        model = GeminiModel()
        assert isinstance(model.client, AsyncOpenAI)
        assert model.client.api_key == "test_key"
        assert str(model.client.base_url) == "http://test_url.com/v1/"
        assert isinstance(model.chat_model, OpenAIChatCompletionsModel)
        assert model.chat_model.model == "gemini-2.5-flash"

def test_gemini_model_initialization_missing_api_key():
    """
    Tests initialization failure when GEMINI_API_KEY is missing.
    """
    with patch.dict(os.environ, {"GEMINI_API_BASE_URL": "http://test_url.com/v1"}, clear=True):
        with pytest.raises(ValueError, match="GEMINI_API_KEY and GEMINI_API_BASE_URL must be set"):
            GeminiModel()

def test_gemini_model_initialization_missing_base_url():
    """
    Tests initialization failure when GEMINI_API_BASE_URL is missing.
    """
    with patch.dict(os.environ, {"GEMINI_API_KEY": "test_key"}, clear=True):
        with pytest.raises(ValueError, match="GEMINI_API_KEY and GEMINI_API_BASE_URL must be set"):
            GeminiModel()

def test_gemini_model_initialization_empty_api_key():
    """
    Tests initialization failure when GEMINI_API_KEY is an empty string.
    """
    with patch.dict(os.environ, {
        "GEMINI_API_KEY": "",
        "GEMINI_API_BASE_URL": "http://test_url.com/v1"
    }, clear=True):
        with pytest.raises(ValueError, match="GEMINI_API_KEY and GEMINI_API_BASE_URL must be set"):
            GeminiModel()

def test_gemini_model_initialization_empty_base_url():
    """
    Tests initialization failure when GEMINI_API_BASE_URL is an empty string.
    """
    with patch.dict(os.environ, {
        "GEMINI_API_KEY": "test_key",
        "GEMINI_API_BASE_URL": ""
    }, clear=True):
        with pytest.raises(ValueError, match="GEMINI_API_KEY and GEMINI_API_BASE_URL must be set"):
            GeminiModel()