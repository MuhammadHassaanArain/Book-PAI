# Implementation Tasks: Urdu Page Translation

**Feature**: Urdu Page Translation
**Branch**: `003-urdu-page-translation`
**Generated**: 2025-12-22
**Spec**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

## Implementation Strategy

**MVP Approach**: Implement User Story 1 (core translation functionality) first, then enhance with toggle functionality (US2) and personalized content support (US3).

**Task Format**: Each task follows the checklist format: `- [ ] T### [P?] [USx?] Description with file path`

**Parallel Execution**: Tasks marked [P] can be executed in parallel if they modify different files/components.

## Dependencies

- User Story 2 (toggle functionality) depends on User Story 1 (core translation)
- User Story 3 (personalized content) depends on User Story 1 (core translation)

## Parallel Execution Examples

- **User Story 1**: Backend API implementation [P] can run in parallel with frontend component development [P]
- **User Story 2**: State management hook [P] can be developed in parallel with UI toggle logic [P]

---

## Phase 1: Setup

- [X] T001 Create backend directory structure: `backend/src/api/`, `backend/src/services/`, `backend/tests/`
- [X] T002 Create frontend directory structure: `src/components/`, `src/pages/hooks/`, `api/`
- [X] T003 Set up Python virtual environment and install dependencies: fastapi, uvicorn, openai, python-dotenv
- [X] T004 Set up environment variables file with OPENAI_API_KEY placeholder

## Phase 2: Foundational Components

- [X] T005 [P] Create backend main application file: `backend/src/main.py`
- [X] T006 [P] Define Pydantic models for TranslationRequest and TranslationResponse: `backend/src/models/translation.py`
- [X] T007 Create frontend API client for translation: `api/translation.js`
- [X] T008 Set up backend configuration with OpenAI API key loading from environment variables

## Phase 3: User Story 1 - Translate Page Content to Urdu

**Goal**: As a logged-in user, I want to translate the current page/chapter content to Urdu on-demand so that I can read the book content in my preferred language without affecting other pages.

**Independent Test**: Can be fully tested by navigating to a page, clicking the "Translate to Urdu" button, and verifying that the page content is replaced with accurate Urdu translation while other pages remain unchanged.

**Acceptance Scenarios**:
1. Given user is on a book chapter page with English content, When user clicks "Translate to Urdu" button, Then page content is replaced with accurate Urdu translation
2. Given user has clicked "Translate to Urdu" button, When translation process completes, Then user sees translated content without navigating away from the page

- [X] T009 [P] [US1] Implement backend translation endpoint: `backend/src/api/translation.py`
- [X] T010 [P] [US1] Create translation service using OpenAI SDK: `backend/src/services/translation_service.py`
- [X] T011 [P] [US1] Create TranslationButton React component: `src/components/TranslationButton.js`
- [X] T012 [P] [US1] Implement content capture logic to extract page content dynamically
- [X] T013 [US1] Integrate TranslationButton with translation API endpoint
- [ ] T014 [US1] Test basic translation functionality with sample page content

## Phase 4: User Story 2 - Toggle Between Languages

**Goal**: As a user who has translated a page to Urdu, I want to toggle back to the original English content so that I can compare translations or read in the original language.

**Independent Test**: Can be tested by translating a page to Urdu and then using a toggle mechanism to return to the original English content.

**Acceptance Scenarios**:
1. Given page has been translated to Urdu, When user activates language toggle, Then page reverts to original English content
2. Given user has toggled between Urdu and English, When user refreshes the page, Then page shows original English content by default

- [X] T015 [P] [US2] Create useTranslation hook to manage translation state: `src/pages/hooks/useTranslation.js`
- [X] T016 [P] [US2] Update TranslationButton component to support toggle functionality
- [X] T017 [US2] Implement logic to preserve original content in component state
- [ ] T018 [US2] Test toggle functionality between English and Urdu content

## Phase 5: User Story 3 - Handle Personalized Content Translation

**Goal**: As a user with personalized book content, I want the translation feature to work with my personalized content so that my customized book experience is available in Urdu as well.

**Independent Test**: Can be tested by ensuring personalized content appears in Urdu after translation, maintaining the personalization elements in the translated output.

**Acceptance Scenarios**:
1. Given user has personalized content on a page, When user clicks "Translate to Urdu" button, Then both original and personalized content are translated to Urdu

- [ ] T019 [P] [US3] Update translation service to handle personalized content appropriately
- [ ] T020 [US3] Modify content capture logic to include personalized content
- [ ] T021 [US3] Test translation functionality with personalized book content

## Phase 6: Polish & Cross-Cutting Concerns

- [X] T022 Add error handling for API failures and rate limits
- [X] T023 Implement loading states and user feedback during translation
- [X] T024 Add validation for translation requests (text length, language codes)
- [X] T025 Handle edge cases like empty content, very large pages, special formatting
- [X] T026 Add caching mechanism to improve performance for repeated translations
- [ ] T027 Update Docusaurus configuration to include translation components
- [ ] T028 Write comprehensive tests for backend translation API
- [ ] T029 Write frontend component tests for TranslationButton
- [ ] T030 Document the translation feature for other developers