import logging
from typing import List, Optional
from pathlib import Path
import glob
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer
from sqlmodel import select
from sqlalchemy.ext.asyncio import AsyncSession

from backend.src.database import get_session
from backend.src.models.personalization_rules import PersonalizationRule, PersonalizationRuleCreate, PersonalizationRuleUpdate
from backend.src.services.personalization_service import PersonalizationService
from backend.src.services.user_service import get_current_user, get_current_user_optional
from backend.src.models.user import User
from backend.src.models.user_profile import UserProfile
from backend.src.services.content_service import ContentService

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/personalize", tags=["personalization"])


def find_chapter_file(docs_path: Path, chapter_path: str) -> Optional[Path]:
    """
    Dynamically find chapter file with flexible path matching.
    Handles:
    - Numbered prefixes (01-, 02-, etc.)
    - Nested folders (quizzes/, try-with-ai/)
    - Various file extensions (.md, .mdx)
    """
    chapter_path_clean = chapter_path.strip("/")

    # Parse path components
    parts = chapter_path_clean.split("/")

    if len(parts) >= 2:
        # Has directory component like "module-ros/intro-to-ros2" or "module-ros/quizzes/quiz-intro"
        dir_part = parts[0]  # e.g., "module-ros"
        remaining_parts = parts[1:]  # e.g., ["intro-to-ros2"] or ["quizzes", "quiz-intro"]
        file_part = parts[-1]  # The actual file name without extension

        # Check if there's a nested subfolder
        if len(remaining_parts) > 1:
            # Nested path like "module-ros/quizzes/quiz-intro-to-ros2"
            subfolder = remaining_parts[0]  # e.g., "quizzes"

            # Try to find the directory with prefix
            dir_patterns = [
                f"[0-9][0-9]-{dir_part}",  # 01-module-ros, 02-module-ros
                dir_part,  # module-ros
            ]

            for dir_pattern in dir_patterns:
                # Look for file in nested subfolder
                search_patterns = [
                    f"{dir_pattern}/{subfolder}/[0-9][0-9]-{file_part}.md",
                    f"{dir_pattern}/{subfolder}/[0-9][0-9]-{file_part}.mdx",
                    f"{dir_pattern}/{subfolder}/{file_part}.md",
                    f"{dir_pattern}/{subfolder}/{file_part}.mdx",
                ]

                for pattern in search_patterns:
                    full_pattern = str(docs_path / pattern)
                    matches = glob.glob(full_pattern)
                    if matches:
                        return Path(matches[0])
        else:
            # Simple nested path like "module-ros/intro-to-ros2"
            dir_patterns = [
                f"[0-9][0-9]-{dir_part}",  # 01-module-ros
                dir_part,  # module-ros
            ]

            for dir_pattern in dir_patterns:
                # Try various file patterns with numbered prefixes
                search_patterns = [
                    f"{dir_pattern}/[0-9][0-9]-{file_part}.md",
                    f"{dir_pattern}/[0-9][0-9]-{file_part}.mdx",
                    f"{dir_pattern}/{file_part}.md",
                    f"{dir_pattern}/{file_part}.mdx",
                ]

                for pattern in search_patterns:
                    full_pattern = str(docs_path / pattern)
                    matches = glob.glob(full_pattern)
                    if matches:
                        return Path(matches[0])
    else:
        # Simple path like "intro"
        file_part = chapter_path_clean

        # Search in root docs folder
        search_patterns = [
            f"[0-9][0-9]-{file_part}.md",
            f"[0-9][0-9]-{file_part}.mdx",
            f"{file_part}.md",
            f"{file_part}.mdx",
        ]

        for pattern in search_patterns:
            full_pattern = str(docs_path / pattern)
            matches = glob.glob(full_pattern)
            if matches:
                return Path(matches[0])

        # Also search in all module directories
        search_patterns = [
            f"[0-9][0-9]-*/[0-9][0-9]-{file_part}.md",
            f"[0-9][0-9]-*/[0-9][0-9]-{file_part}.mdx",
            f"[0-9][0-9]-*/{file_part}.md",
            f"[0-9][0-9]-*/{file_part}.mdx",
        ]

        for pattern in search_patterns:
            full_pattern = str(docs_path / pattern)
            matches = glob.glob(full_pattern)
            if matches:
                return Path(matches[0])

    return None


@router.get("/{chapter_path:path}")
async def get_personalized_content(
    chapter_path: str,
    current_user: Optional[User] = Depends(get_current_user_optional),
    session: AsyncSession = Depends(get_session)
):
    """
    Get personalized chapter content based on user profile
    """
    try:
        logger.info("=== Personalization Request Start ===")
        logger.info(f"chapter_path received: {chapter_path}")
        logger.info(f"current_user: {current_user.id if current_user else 'None'}")

        # ✅ Normalize chapter path safely
        chapter_path_clean = chapter_path.strip("/")
        chapter_key = chapter_path_clean.split("/")[-1] if "/" in chapter_path_clean else chapter_path_clean

        # ✅ Resolve docs path once
        BASE_DIR = Path(__file__).resolve().parents[4]
        DOCS_PATH = BASE_DIR / "frontend/docs"
        logger.info(f"DOCS_PATH: {DOCS_PATH}")

        # ✅ Use dynamic file finder
        found_file = find_chapter_file(DOCS_PATH, chapter_path_clean)

        if found_file is None:
            logger.error(f"Content load failed - no matching file found for: {chapter_path_clean}")
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Chapter content not found for path: {chapter_path}"
            )

        logger.info(f"Found file: {found_file}")

        # ✅ Load original content from found file
        try:
            original_content = found_file.read_text(encoding='utf-8')
            logger.info(f"Content loaded, length: {len(original_content)}")
        except Exception as e:
            logger.error(f"Failed to read file {found_file}: {e}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Failed to read chapter content: {str(e)}"
            )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"ERROR in get_personalized_content: {e}")
        import traceback
        traceback.print_exc()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing request: {str(e)}"
        )

    # ✅ If user not logged in → return original content
    if not current_user:
        return {
            "chapter_path": chapter_path,
            "personalized_content": original_content,
            "user_profile": None,
            "personalization_applied": False
        }

    # ✅ Fetch user profile
    statement = select(UserProfile).where(UserProfile.user_id == current_user.id)
    result = await session.execute(statement)
    user_profile = result.first()

    # Debug logging for user_profile
    if user_profile:
        logger.info(f"User profile fetched successfully for user {current_user.id}")
        logger.debug(f"User profile type: {type(user_profile)}")
        logger.debug(f"User profile attributes: {dir(user_profile)}")
        if hasattr(user_profile, 'user_id'):
            logger.debug(f"User profile user_id: {user_profile.user_id}")
        else:
            logger.error(f"User profile missing user_id attribute for user {current_user.id}")
    else:
        logger.warning(f"No user profile found for user {current_user.id}")

    if not user_profile:
        return {
            "chapter_path": chapter_key,
            "personalized_content": original_content,
            "user_profile": None,
            "personalization_applied": False
        }

    # Additional safety check for user_profile attributes
    if not hasattr(user_profile, 'user_id'):
        # This shouldn't happen, but handle it gracefully
        logger.error(f"User profile missing user_id attribute for user {current_user.id}")
        return {
            "chapter_path": chapter_key,
            "personalized_content": original_content,
            "user_profile": None,
            "personalization_applied": False
        }

    # ✅ Personalization service (removed DOCS_PATH - not needed in constructor)
    service = PersonalizationService(session)

    try:
        personalized_content = await service.get_personalized_content(
            chapter_path=chapter_key,
            user_profile=user_profile
        )

        return {
            "chapter_path": chapter_key,
            "personalized_content": personalized_content,
            "user_profile": {
                "software_background": user_profile.software_background,
                "hardware_background": user_profile.hardware_background,
                "experience_level": user_profile.experience_level
            },
            "personalization_applied": personalized_content != original_content
        }

    except Exception as e:
        logger.error(f"Personalization error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error personalizing content"
        )


from pydantic import BaseModel


class PreviewPersonalizationRequest(BaseModel):
    chapter_path: str
    profile_data: dict


@router.post("/preview")
async def preview_personalized_content(
    request_data: PreviewPersonalizationRequest,
    current_user: User = Depends(get_current_user),
    session: AsyncSession = Depends(get_session)
):
    """
    Preview personalized content with custom profile data (admin/moderator only)
    """
    # Check if user is admin/moderator
    if not current_user.is_superuser:  # Assuming is_superuser indicates admin
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Preview functionality requires admin/moderator privileges"
        )

    # Validate profile_data structure
    profile_data = request_data.profile_data
    required_fields = {"software_background", "hardware_background", "experience_level"}
    if not all(field in profile_data for field in required_fields):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Profile data must contain: {required_fields}"
        )

    # Create a temporary user profile from provided data for preview
    from backend.src.models.user_profile import UserProfile
    temp_profile = UserProfile(
        user_id=current_user.id,  # Use current user ID for the session
        software_background=profile_data["software_background"],
        hardware_background=profile_data["hardware_background"],
        experience_level=profile_data.get("experience_level", "beginner")
    )

    # Use personalization service to get personalized content with temp profile
    service = PersonalizationService(session)
    try:
        personalized_content = await service.get_personalized_content(
            chapter_path=request_data.chapter_path,
            user_profile=temp_profile
        )

        return {
            "chapter_path": request_data.chapter_path,
            "personalized_content": personalized_content,
            "applied_profile": profile_data,
            "personalization_applied": personalized_content != await service.get_original_content(request_data.chapter_path)
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error generating preview: {str(e)}"
        )


@router.get("/rules")
async def get_personalization_rules(
    current_user: User = Depends(get_current_user),
    session: AsyncSession = Depends(get_session)
):
    """
    List all active personalization rules (admin/moderator only)
    """
    # Check if user is admin/moderator
    if not current_user.is_superuser:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access to personalization rules requires admin/moderator privileges"
        )

    statement = select(PersonalizationRule).where(PersonalizationRule.is_active == True)
    result = await session.execute(statement)
    rules = result.all()

    return {"rules": rules}


@router.put("/rules/{rule_id}")
async def update_personalization_rule(
    rule_id: int,
    rule_update: PersonalizationRuleCreate,  # Using Create model since it has all necessary fields
    current_user: User = Depends(get_current_user),
    session: AsyncSession = Depends(get_session)
):
    """
    Update an existing personalization rule (admin/moderator only)
    """
    # Check if user is admin/moderator
    if not current_user.is_superuser:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Updating personalization rules requires admin/moderator privileges"
        )

    # Fetch the existing rule
    rule = await session.get(PersonalizationRule, rule_id)
    if not rule:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Rule not found"
        )

    # Update the rule with the new data
    rule.content_path = rule_update.content_path
    rule.user_profile_criteria = rule_update.user_profile_criteria
    rule.content_variants = rule_update.content_variants
    rule.description = rule_update.description
    rule.priority = rule_update.priority
    rule.is_active = rule_update.is_active

    session.add(rule)
    await session.commit()
    await session.refresh(rule)

    # Log the change for audit purposes
    logger.info(f"Personalization rule {rule_id} updated by user {current_user.id}")

    return rule