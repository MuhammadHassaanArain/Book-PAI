from pydantic import BaseModel
from typing import Optional
from datetime import datetime

class TranslationRequest(BaseModel):
    text: str
    sourceLanguage: str = "en"
    targetLanguage: str = "ur"
    pageId: Optional[str] = None

class TranslationResponse(BaseModel):
    translated_text: str
    source_text: str
    source_language: str
    target_language: str
    translation_time: str
    success: bool
    error: Optional[str] = None