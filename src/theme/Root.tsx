import React from 'react';
import { ChatProvider } from '../components/chatbot/ChatContext';
import ChatBotButton from '../components/chatbot/ChatBotButton';
import ChatBotPanel from '../components/chatbot/ChatBotPanel';

function Root({ children }: { children: React.ReactNode }) {
  return (
    <ChatProvider>
      {children}
      <ChatBotButton />
      <ChatBotPanel />
    </ChatProvider>
  );
}

export default Root;