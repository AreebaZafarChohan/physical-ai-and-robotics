from fastapi import APIRouter, HTTPException, Depends, Query
from typing import Optional
from backend.src.api.translation_service import (
    TranslationService,
    TranslationRequest,
    TranslationStatusResponse,
    AvailableTranslationsResponse,
    UpdateTranslationRequest,
    LocalesResponse,
    UserPreferenceRequest,
    UserPreferenceResponse
)

router = APIRouter(prefix="/translate", tags=["translation"])
translation_service = TranslationService()

@router.post("/")
async def trigger_translation(request: TranslationRequest):
    """Initiates the translation process for English to Urdu"""
    try:
        result = await translation_service.trigger_translation(request)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/{job_id}")
async def get_translation_status(job_id: str):
    """Gets the status of a specific translation job"""
    try:
        result = await translation_service.get_translation_status(job_id)
        return result
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/status")
async def get_bulk_translation_status(
    limit: int = Query(10, ge=1, le=100),
    status: Optional[str] = Query(None)
):
    """Gets the status of all recent translation jobs"""
    try:
        result = await translation_service.get_bulk_translation_status(limit, status)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/content/translations")
async def get_available_translations(
    file_path: Optional[str] = Query(None)
):
    """Lists all available translations for content"""
    try:
        result = await translation_service.get_available_translations(file_path)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.put("/content/translation")
async def update_translation(request: UpdateTranslationRequest):
    """Updates a specific translation file"""
    try:
        result = await translation_service.update_translation(request)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/locales")
async def get_available_locales():
    """Gets all supported locales for the site"""
    try:
        result = await translation_service.get_available_locales()
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/user/preference")
async def set_user_preference(request: UserPreferenceRequest):
    """Sets the user's language preference"""
    try:
        result = await translation_service.set_user_preference(request)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))