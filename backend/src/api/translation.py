from fastapi import APIRouter, HTTPException
from backend.src.models.translation import TranslationRequest, TranslationResponse
from backend.src.services.translation_service import TranslationService

router = APIRouter()

@router.post("/translate", response_model=TranslationResponse)
async def translate_endpoint(request: TranslationRequest) -> TranslationResponse:
    """
    Translates text from source language to target language
    """
    # Validate the request
    is_valid, error_msg = await TranslationService.validate_translation_request(request)
    if not is_valid:
        raise HTTPException(status_code=400, detail=error_msg)

    # Perform the translation
    result = await TranslationService.translate_text(request)

    if not result.success:
        raise HTTPException(status_code=500, detail=result.error or "Translation failed")

    return result