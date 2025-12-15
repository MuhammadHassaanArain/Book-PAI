import React from 'react';
import { ChatMessage as ChatMessageType } from './types';
import './chatbot.css';

interface ChatMessageProps {
  message: ChatMessageType;
}

const ChatMessage: React.FC<ChatMessageProps> = ({ message }) => {
  return (
    <div className={`chat-message ${message.sender}`}>
      <div className="chat-message-text">{message.text}</div>
    </div>
  );
};

export default ChatMessage;