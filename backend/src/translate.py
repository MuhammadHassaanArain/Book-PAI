from fastapi import APIRouter
from .models.translation import TranslationRequest, TranslationResponse
from .services.translation_service import TranslationService  # your service logic
from datetime import datetime

router = APIRouter(
    prefix="/translate",
    tags=["translation"]
)

@router.post("/", response_model=TranslationResponse)
async def translate(request: TranslationRequest):
    # Optional: validate request
    valid, error_msg = await TranslationService.validate_translation_request(request)
    if not valid:
        return TranslationResponse(
            translated_text="",
            source_text=request.text,
            source_language=request.sourceLanguage,
            target_language=request.targetLanguage,
            translation_time=datetime.utcnow().isoformat(),
            success=False,
            error=error_msg
        )
    
    # Call translation service
    result = await TranslationService.translate_text(request)
    return result
