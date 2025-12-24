import os
import logging
import asyncio
from datetime import datetime
from typing import Optional
from dotenv import load_dotenv
from agents import (Agent,Runner,OpenAIChatCompletionsModel,AsyncOpenAI,RunConfig,)
from backend.src.models.translation import (TranslationRequest,TranslationResponse,)
from backend.src.utils.cache import translation_cache

load_dotenv()
logger = logging.getLogger(__name__)

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY_TRANSLATE")

if not GEMINI_API_KEY:
    logger.warning(
        "GEMINI_API_KEY is not configured. Translation will fail."
    )
gemini_client = AsyncOpenAI(
    api_key=GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)
translation_model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=gemini_client,
)
run_config = RunConfig(
    model=translation_model,
    model_provider=gemini_client,
    tracing_disabled=True,
)

translation_agent = Agent(
    name="translation-agent",
    instructions=(
        "You are a professional translation agent.\n"
        "Translate ONLY the provided text.\n"
        "Preserve formatting, meaning, and tone.\n"
        "Do NOT explain.\n"
        "Return ONLY the translated text."
    ),
)


class TranslationService:
    @staticmethod
    async def translate_text(
        translation_request: TranslationRequest
    ) -> TranslationResponse:
        """
        Translate page-level text using OpenAI Agent SDK + Gemini
        """
        cached = translation_cache.get(
            translation_request.text,
            translation_request.sourceLanguage,
            translation_request.targetLanguage,
        )

        if cached:
            logger.info("Returning cached translation")
            return TranslationResponse(
                translated_text=cached["translated_text"],
                source_text=translation_request.text,
                source_language=translation_request.sourceLanguage,
                target_language=translation_request.targetLanguage,
                translation_time=cached["translation_time"],
                success=True,
            )

        if not GEMINI_API_KEY or GEMINI_API_KEY == "your_gemini_api_key_here":
            # For testing purposes, return a mock translation
            logger.warning("Using mock translation for testing - GEMINI_API_KEY not configured")
            mock_translation = f"URDU MOCK: {translation_request.text}"
            response = TranslationResponse(
                translated_text=mock_translation,
                source_text=translation_request.text,
                source_language=translation_request.sourceLanguage,
                target_language=translation_request.targetLanguage,
                translation_time=datetime.utcnow().isoformat(),
                success=True,
            )

            # Cache the mock result
            translation_cache.set(
                translation_request.text,
                translation_request.sourceLanguage,
                translation_request.targetLanguage,
                {
                    "translated_text": mock_translation,
                    "translation_time": response.translation_time,
                },
            )

            return response

        try:
           
            prompt = f"""
Translate the following text from
{translation_request.sourceLanguage}
to
{translation_request.targetLanguage}.

TEXT:
{translation_request.text}
""".strip()

       
         

            result = await asyncio.to_thread(
                Runner.run_sync,
                translation_agent,
                prompt,
                run_config=run_config,
            )

            translated_text = result.final_output.strip()
            response = TranslationResponse(
                translated_text=translated_text,
                source_text=translation_request.text,
                source_language=translation_request.sourceLanguage,
                target_language=translation_request.targetLanguage,
                translation_time=datetime.utcnow().isoformat(),
                success=True,
            )

            translation_cache.set(
                translation_request.text,
                translation_request.sourceLanguage,
                translation_request.targetLanguage,
                {
                    "translated_text": translated_text,
                    "translation_time": response.translation_time,
                },
            )

            return response

        except Exception as e:
            logger.error(f"Translation agent failed: {e}")
            return TranslationResponse(
                translated_text="",
                source_text=translation_request.text,
                source_language=translation_request.sourceLanguage,
                target_language=translation_request.targetLanguage,
                translation_time=datetime.utcnow().isoformat(),
                success=False,
                error=str(e),
            )

    @staticmethod
    async def validate_translation_request(
        translation_request: TranslationRequest
    ) -> tuple[bool, Optional[str]]:

        if not translation_request.text.strip():
            return False, "Text to translate cannot be empty"

        if len(translation_request.text) > 10_000:
            return False, "Text too long (max 10,000 characters)"

        supported_languages = ["en", "ur"]

        if translation_request.sourceLanguage not in supported_languages:
            return False, (
                f"Unsupported source language: "
                f"{translation_request.sourceLanguage}"
            )

        if translation_request.targetLanguage not in supported_languages:
            return False, (
                f"Unsupported target language: "
                f"{translation_request.targetLanguage}"
            )

        return True, None
