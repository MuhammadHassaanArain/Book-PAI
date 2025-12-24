# Quickstart Guide: Urdu Page Translation

## Prerequisites
- Node.js 16+ (for Docusaurus frontend)
- Python 3.10+ (for FastAPI backend)
- OpenAI API key
- Git

## Setup Backend

1. Install Python dependencies:
```bash
pip install fastapi uvicorn openai python-dotenv
```

2. Create the translation endpoint at `backend/src/main.py`:
```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import openai
import os
from datetime import datetime

app = FastAPI()

class TranslationRequest(BaseModel):
    text: str
    sourceLanguage: str = "en"
    targetLanguage: str = "ur"

class TranslationResponse(BaseModel):
    translated_text: str
    source_text: str
    source_language: str
    target_language: str
    translation_time: str
    success: bool

@app.post("/api/translate", response_model=TranslationResponse)
async def translate_text(request: TranslationRequest):
    try:
        # Configure OpenAI API
        openai.api_key = os.getenv("OPENAI_API_KEY")

        # Create translation using OpenAI
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are a professional translator. Translate the following text to Urdu while preserving context and meaning. Respond with only the translated text."},
                {"role": "user", "content": request.text}
            ],
            max_tokens=len(request.text) * 2
        )

        translated_text = response.choices[0].message.content.strip()

        return TranslationResponse(
            translated_text=translated_text,
            source_text=request.text,
            source_language=request.sourceLanguage,
            target_language=request.targetLanguage,
            translation_time=datetime.utcnow().isoformat(),
            success=True
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

3. Run the backend:
```bash
uvicorn backend.src.main:app --reload --port 8000
```

## Setup Frontend Component

1. Create TranslationButton component:
```javascript
// src/components/TranslationButton.js
import React, { useState } from 'react';
import axios from 'axios';

const TranslationButton = ({ pageContent, onPageUpdate }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [originalContent, setOriginalContent] = useState(pageContent);

  const handleTranslate = async () => {
    setIsTranslating(true);
    try {
      const response = await axios.post('/api/translate', {
        text: pageContent,
        sourceLanguage: 'en',
        targetLanguage: 'ur'
      });

      onPageUpdate(response.data.translated_text);
      setIsTranslated(true);
    } catch (error) {
      console.error('Translation failed:', error);
    } finally {
      setIsTranslating(false);
    }
  };

  const handleToggle = () => {
    if (isTranslated) {
      onPageUpdate(originalContent);
      setIsTranslated(false);
    } else {
      handleTranslate();
    }
  };

  return (
    <button
      onClick={handleToggle}
      disabled={isTranslating}
      className="translation-button"
    >
      {isTranslating
        ? 'Translating...'
        : isTranslated
          ? 'Show in English'
          : 'Translate to Urdu'}
    </button>
  );
};

export default TranslationButton;
```

## Environment Variables
Create a `.env` file in the backend directory:
```
OPENAI_API_KEY=your_openai_api_key_here
```