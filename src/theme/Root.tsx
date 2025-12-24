import React from 'react';
import { ChatProvider } from '../components/chatbot/ChatContext';
import ChatBotButton from '../components/chatbot/ChatBotButton';
import ChatBotPanel from '../components/chatbot/ChatBotPanel';
import PageTranslationButton from '../components/PageTranslationButton';

function Root({ children }: { children: React.ReactNode }) {
  return (
    <ChatProvider>
      {children}
      <PageTranslationButton />
      <ChatBotButton />
      <ChatBotPanel />
    </ChatProvider>
  );
}

export default Root;