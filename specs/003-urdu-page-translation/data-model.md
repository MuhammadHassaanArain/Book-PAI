# Data Model: Urdu Page Translation

## Entities

### PageContent
- **Description**: The text content of a single book chapter/page that can be translated between English and Urdu
- **Fields**:
  - id: string (unique identifier for the page)
  - originalContent: string (the original English content)
  - translatedContent: string (the Urdu translation, optional)
  - contentType: string (format of the content - markdown, html, etc.)
  - lastTranslated: datetime (timestamp of last translation)

### TranslationRequest
- **Description**: The data structure containing English text to be translated to Urdu
- **Fields**:
  - text: string (the English text to translate)
  - sourceLanguage: string (default: "en", source language code)
  - targetLanguage: string (default: "ur", target language code)
  - pageId: string (optional, identifier of the source page)

### TranslationResponse
- **Description**: The data structure containing the Urdu translation of the original English text
- **Fields**:
  - translatedText: string (the Urdu translation)
  - sourceText: string (the original text that was translated)
  - sourceLanguage: string (source language code)
  - targetLanguage: string (target language code)
  - translationTime: datetime (timestamp of translation completion)
  - success: boolean (whether translation was successful)

### UserSession
- **Description**: The logged-in user state that enables access to translation features
- **Fields**:
  - userId: string (unique identifier for the user)
  - isLoggedIn: boolean (whether user is authenticated)
  - preferences: object (user preferences including language settings)
  - permissions: array (list of permissions for the user)