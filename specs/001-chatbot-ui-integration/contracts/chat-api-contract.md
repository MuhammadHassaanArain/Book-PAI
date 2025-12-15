# API Contract: Chatbot Integration

## Overview
This document defines the API contract between the frontend chatbot UI and the backend AI assistant service.

## Endpoints

### POST /api/chat
Send a message to the AI assistant and receive a response.

#### Request
```json
{
  "message": "Hello, how can you help me with this documentation?",
  "sessionId": "optional-session-id-for-context"
}
```

**Headers**:
- Content-Type: application/json

**Parameters**:
- message (string, required): The user's message to send to the AI assistant
- sessionId (string, optional): Session identifier to maintain conversation context

#### Response
**Success (200 OK)**:
```json
{
  "response": "I can help explain the concepts in this documentation. What specifically would you like to know?",
  "status": "success",
  "sessionId": "session-id-for-context-continuity"
}
```

**Error (400 Bad Request)**:
```json
{
  "error": "Invalid request format",
  "status": "error"
}
```

**Server Error (500 Internal Server Error)**:
```json
{
  "error": "AI service temporarily unavailable",
  "status": "error"
}
```

#### Error Codes
- 400: Invalid request format (malformed JSON, missing required fields)
- 500: Server error (AI service unavailable, processing error)

## Message Format
- Messages must be UTF-8 encoded strings
- Maximum message length: 2000 characters
- Minimum message length: 1 character

## Session Management
- If no sessionId is provided, a new session is created
- The backend returns a sessionId that should be used for subsequent messages in the same conversation
- Sessions may expire after 30 minutes of inactivity

## Rate Limiting
- Maximum 10 requests per minute per client
- Exceeding rate limit returns 429 status code