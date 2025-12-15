# Quickstart: Docusaurus Chatbot UI Integration

## Prerequisites
- Node.js 18+ installed
- Docusaurus project already set up
- Backend API `/api/chat` endpoint available

## Installation Steps

### 1. Create Component Directory
```bash
mkdir -p src/components/chatbot
```

### 2. Install Dependencies
```bash
npm install react-icons  # For chat icons
# If using TypeScript
npm install --save-dev @types/react
```

### 3. Create Chat Components
Create the following files in `src/components/chatbot/`:

- `ChatBotButton.tsx` - Floating button component
- `ChatBotPanel.tsx` - Main chat panel
- `ChatMessage.tsx` - Individual message component
- `ChatContext.tsx` - State management context
- `chatbot.css` - Styling

### 4. Integrate with Docusaurus
Add the chatbot button to your Docusaurus root layout by importing it in `src/theme/Root.tsx`:

```tsx
import ChatBotButton from '@site/src/components/chatbot/ChatBotButton';

export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <>
      {children}
      <ChatBotButton />
    </>
  );
}
```

### 5. Verify API Connection
Test that the `/api/chat` endpoint is accessible and returns expected responses:

```bash
curl -X POST http://localhost:3000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello"}'
```

## Running the Application
```bash
npm run start
```

The chatbot should appear as a floating button on the bottom-right of all pages.

## Configuration Options
- Modify `chatbot.css` to adjust colors and styling to match your theme
- Adjust positioning and animation settings in the component files
- Update API endpoint URL if hosted separately

## Testing
- Verify the chat button appears on all pages
- Test sending and receiving messages
- Confirm message history persists across page navigation
- Test responsive behavior on mobile devices