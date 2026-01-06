"""
Content processing utilities for personalization and translation.
"""

import re
from typing import Dict, List, Optional


def extract_user_preferences(user_data: Dict) -> Dict:
    """
    Extract user preferences from user data.

    Args:
        user_data: Dictionary containing user information

    Returns:
        Dictionary of extracted preferences
    """
    preferences = {
        "language": user_data.get("language", "en"),
        "difficulty_level": user_data.get("difficulty_level", "beginner"),
        "learning_style": user_data.get("learning_style", "visual"),
        "topics_of_interest": user_data.get("topics_of_interest", [])
    }
    return preferences


def personalize_content(content: str, preferences: Dict) -> str:
    """
    Personalize content based on user preferences.

    Args:
        content: Original content string
        preferences: User preferences dictionary

    Returns:
        Personalized content string
    """
    # Add difficulty indicators based on user level
    level = preferences.get("difficulty_level", "beginner")

    if level == "beginner":
        content = f"🎓 **Beginner Level**\n\n{content}"
    elif level == "intermediate":
        content = f"📚 **Intermediate Level**\n\n{content}"
    elif level == "advanced":
        content = f"🚀 **Advanced Level**\n\n{content}"

    return content


def translate_content(content: str, target_language: str) -> str:
    """
    Translate content to target language.
    Note: This is a placeholder. Integrate with actual translation service.

    Args:
        content: Content to translate
        target_language: Target language code

    Returns:
        Translated content
    """
    # Placeholder for translation logic
    # In production, integrate with Google Translate API or similar
    return content


def extract_code_blocks(content: str) -> List[Dict]:
    """
    Extract code blocks from markdown content.

    Args:
        content: Markdown content string

    Returns:
        List of dictionaries containing code blocks
    """
    code_pattern = r"```(\w+)?\n(.*?)```"
    matches = re.findall(code_pattern, content, re.DOTALL)

    code_blocks = []
    for lang, code in matches:
        code_blocks.append({
            "language": lang or "text",
            "code": code.strip()
        })

    return code_blocks


def add_interactive_elements(content: str, preferences: Dict) -> str:
    """
    Add interactive elements to content based on learning style.

    Args:
        content: Original content
        preferences: User preferences

    Returns:
        Content with interactive elements
    """
    learning_style = preferences.get("learning_style", "visual")

    if learning_style == "hands-on":
        content += "\n\n💻 **Try it yourself**: Run the code examples in your local environment."
    elif learning_style == "visual":
        content += "\n\n👁️ **Visual learners**: Check out the diagrams and animations."
    elif learning_style == "reading":
        content += "\n\n📖 **Deep dive**: Read the detailed explanations below."

    return content


def sanitize_content(content: str) -> str:
    """
    Sanitize content to remove harmful scripts or tags.

    Args:
        content: Content to sanitize

    Returns:
        Sanitized content
    """
    # Remove script tags
    content = re.sub(r"<script[^>]*>.*?</script>", "", content, flags=re.DOTALL | re.IGNORECASE)

    # Remove iframe tags
    content = re.sub(r"<iframe[^>]*>.*?</iframe>", "", content, flags=re.DOTALL | re.IGNORECASE)

    return content


def format_for_locale(content: str, locale: str) -> str:
    """
    Format content for specific locale (RTL/LTR).

    Args:
        content: Content to format
        locale: Locale code (e.g., 'ur', 'ar', 'en')

    Returns:
        Formatted content
    """
    rtl_languages = ["ur", "ar", "fa", "he"]

    if locale in rtl_languages:
        # Add RTL markers or formatting
        content = f'<div dir="rtl">{content}</div>'

    return content
