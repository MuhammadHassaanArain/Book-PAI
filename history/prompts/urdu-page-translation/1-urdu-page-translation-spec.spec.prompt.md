---
id: 1
title: "urdu-page-translation-spec"
stage: "spec"
date_iso: "2025-12-22"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "urdu-page-translation"
branch: "003-urdu-page-translation"
user: "Claude Code"
command: "/sp.specify"
labels: ["specification", "translation", "urdu", "frontend", "backend"]
links:
  spec: "specs/003-urdu-page-translation/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "specs/003-urdu-page-translation/spec.md"
  - "specs/003-urdu-page-translation/checklists/requirements.md"
tests_yaml:
  - "Specification quality checklist completed"
  - "User scenarios defined"
  - "Functional requirements specified"
  - "Success criteria established"
---

## Prompt

```
---
name: "page-translation-agent"
description: "On-demand per-page Urdu translation for book chapters using OpenAI Agent SDK and FastAPI."
version: "1.0.0"
---

# Page Translation Feature Specification

## Purpose
Enable logged-in users to translate the content of a single chapter/page to Urdu on-demand without affecting the rest of the book.

## Scope
- Only translates the content of the **current page/chapter** when user clicks the "Translate to Urdu" button.
- Works with **personalized content** if available.
- Integrates with **FastAPI backend** and **OpenAI Agent SDK**.

## Functional Requirements

- **Frontend**
  - Add a "Translate to Urdu" button at the start of each chapter/page.
  - Capture the current page content dynamically when the button is clicked.
  - Send page content via POST request to `/api/translate` endpoint.
  - Update the page content with translated text returned from backend.
  - Optionally, allow toggling back to English.

- **Backend (FastAPI)**
  - Create a POST endpoint `/api/translate` accepting JSON `{ text: string }`.
  - Forward text to a Translation Agent using OpenAI Agent SDK.
  - Return translated Urdu text as JSON `{ translated_text: string }`.
  - Optional: Cache translations per page to reduce repeated API calls.

- **Translation Agent (OpenAI)**
  - Receive English text and translate it to Urdu.
  - Ensure translation is accurate and preserves context.
  - Optionally integrate with personalized content logic.

## Non-Functional Requirements
- Fast response time (<3 seconds for typical page length).
- Scalable to multiple simultaneous users.
- Modular and reusable agent design for future languages.

## Integration
- Frontend: Docusaurus React pages.
- Backend: FastAPI + Neon Postgres for caching (optional).
- Agent: OpenAI Agent SDK handling the translation logic.

## User Flow
1. User navigates to a chapter page.
2. Clicks "Translate to Urdu" button.
3. Frontend sends page content to `/api/translate`.
4. Backend calls Translation Agent.
5. Agent returns Urdu text.
6. Frontend replaces page content with translated text.
7. Optional toggle back to English restores original content.

## Success Criteria
- Clicking the button only translates the current page content.
- Translations are accurate and readable in Urdu.
- System handles multiple pages and repeated requests efficiently.
```

## Response

Created comprehensive feature specification for Urdu page translation. The specification includes:

- User scenarios and testing flows with prioritized user stories
- Functional requirements defining the core functionality
- Key entities involved in the translation process
- Success criteria with measurable outcomes

The specification was validated against a quality checklist to ensure it:
- Contains no implementation details (technology-agnostic)
- Focuses on user value and business needs
- Has testable and unambiguous requirements
- Includes measurable success criteria
- Identifies edge cases and acceptance scenarios

The feature is now ready for planning phase (/sp.plan).