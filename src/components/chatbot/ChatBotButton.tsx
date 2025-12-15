import React from 'react';
import { useChat } from './ChatContext';
import './chatbot.css';

const ChatBotButton: React.FC = () => {
  const { uiState, togglePanel } = useChat();

  return (
    <button
      className={`chatbot-button ${uiState.isPanelOpen ? 'open' : ''}`}
      onClick={togglePanel}
      aria-label={uiState.isPanelOpen ? 'Close chat' : 'Open chat'}
    >
      {uiState.isPanelOpen ? '✕' : '💬'}
    </button>
  );
};

export default ChatBotButton;