# Data Model: Docusaurus Chatbot UI Integration

## Entities

### ChatMessage
Represents a single message in the chat conversation.

**Fields**:
- id: string (unique identifier for the message)
- text: string (the content of the message)
- sender: 'user' | 'bot' (indicates who sent the message)
- timestamp: Date (when the message was sent/received)
- status: 'sent' | 'delivered' | 'error' (delivery status for user messages)

**Validation**:
- text must be 1-2000 characters
- sender must be either 'user' or 'bot'
- timestamp must be a valid date/time

### ChatSession
Represents the current conversation context containing multiple chat messages.

**Fields**:
- id: string (unique identifier for the session)
- messages: ChatMessage[] (array of messages in chronological order)
- createdAt: Date (when the session was started)
- lastActive: Date (when the last message was sent/received)
- isActive: boolean (whether the chat panel is currently open)

**Validation**:
- messages array length must be 0-1000
- createdAt must be a valid date/time
- lastActive must be a valid date/time

### ChatUIState
Represents the visibility and position state of the chat panel and button.

**Fields**:
- isPanelOpen: boolean (whether the chat panel is currently displayed)
- isMinimized: boolean (whether the chat panel is minimized)
- position: { x: number, y: number } (coordinates for panel positioning)
- unreadCount: number (number of unread messages)

**Validation**:
- isPanelOpen must be boolean
- isMinimized must be boolean
- position coordinates must be non-negative numbers
- unreadCount must be 0-999

## State Transitions

### ChatMessage Status Transitions
- Initial: 'sent' (when user sends message)
- On API success: 'delivered'
- On API error: 'error'

### ChatSession State Transitions
- New session: isActive = true
- Panel closed: isActive = false
- After 30 minutes of inactivity: may be archived (implementation dependent)

### ChatUIState Transitions
- Button click when closed: isPanelOpen = true
- Close button click: isPanelOpen = false
- New bot message when panel closed: unreadCount += 1
- Panel opened: unreadCount = 0

## Storage Schema

### localStorage Structure
```
{
  "chatSessions": {
    "[sessionId]": {
      "id": string,
      "messages": [ChatMessage],
      "createdAt": string (ISO date),
      "lastActive": string (ISO date),
      "isActive": boolean
    }
  },
  "uiState": {
    "isPanelOpen": boolean,
    "isMinimized": boolean,
    "position": { "x": number, "y": number },
    "unreadCount": number
  },
  "lastSessionId": string
}
```

## API Data Contracts

### Request to /api/chat
```
{
  "message": string,
  "sessionId"?: string (optional, for context continuity)
}
```

### Response from /api/chat
```
{
  "response": string,
  "status": "success" | "error",
  "sessionId": string (for context continuity)
}
```