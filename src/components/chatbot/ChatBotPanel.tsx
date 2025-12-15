import React, { useState, useEffect, useRef } from 'react';
import { useChat } from './ChatContext';
import { chatAPI } from './api';
import { ChatMessage as ChatMessageType } from './types';
import './chatbot.css';

const ChatBotPanel: React.FC = () => {
  const { uiState, togglePanel, session, addMessage, clearMessages } = useChat();
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [session?.messages, isLoading]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    try {
      setIsLoading(true);

      // Add user message immediately
      const userMessage: Omit<ChatMessageType, 'id' | 'timestamp'> = {
        text: inputValue,
        sender: 'user',
        status: 'sent'
      };
      addMessage(userMessage);

      // Clear input
      setInputValue('');

      // Get bot response
      const response = await chatAPI.sendMessage({
        message: inputValue,
        sessionId: session?.id
      });

      if (response.status === 'success' && response.response) {
        const botMessage: Omit<ChatMessageType, 'id' | 'timestamp'> = {
          text: response.response,
          sender: 'bot'
        };
        addMessage(botMessage);
      } else {
        const errorMessage: Omit<ChatMessageType, 'id' | 'timestamp'> = {
          text: response.response || 'Sorry, I encountered an error. Please try again.',
          sender: 'bot'
        };
        addMessage(errorMessage);
      }
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: Omit<ChatMessageType, 'id' | 'timestamp'> = {
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot'
      };
      addMessage(errorMessage);
    } finally {
      setIsLoading(false);
    }
  };

  const handleClearMessages = () => {
    if (window.confirm('Are you sure you want to clear all messages?')) {
      addMessage({
        text: 'Chat history cleared',
        sender: 'system'
      });
      // Add a small delay to show the system message before clearing
      setTimeout(() => {
        clearMessages();
      }, 500);
    }
  };

  return (
    <div
      className={`chatbot-panel ${uiState.isPanelOpen ? 'show' : ''}`}
      style={{ display: uiState.isPanelOpen ? 'flex' : 'none' }}
      role="dialog"
      aria-modal="true"
      aria-label="Chat with assistant"
    >
      <div className="chatbot-header" role="banner">
        <span>Assistant</span>
        <div className="chatbot-header-buttons">
          <button
            className="chatbot-clear-button"
            onClick={handleClearMessages}
            aria-label="Clear chat"
          >
            🗑️
          </button>
          <button
            className="chatbot-close-button"
            onClick={togglePanel}
            aria-label="Close chat"
          >
            ✕
          </button>
        </div>
      </div>
      <div
        className="chatbot-body"
        role="log"
        aria-live="polite"
        aria-relevant="additions"
      >
        {session?.messages && session.messages.length > 0 ? (
          session.messages.map((msg) => (
            <div
              key={msg.id}
              className={`chat-message ${msg.sender}`}
              role="listitem"
              aria-label={`${msg.sender === 'user' ? 'You said' : msg.sender === 'system' ? 'System message' : 'Assistant said'}: ${msg.text}`}
            >
              <div className="chat-message-text">{msg.text}</div>
            </div>
          ))
        ) : (
          <div className="chat-message bot" role="listitem">
            <div className="chat-message-text">Hello! How can I help you today?</div>
          </div>
        )}
        {isLoading && (
          <div className="chat-message bot" role="status" aria-live="polite">
            <div className="chat-message-text typing-indicator">Typing...</div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>
      <div className="chatbot-footer">
        <form onSubmit={handleSubmit} className="chat-input-form">
          <input
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder="Type your message..."
            className="chat-input"
            disabled={isLoading}
            aria-label="Type your message"
            role="textbox"
            aria-multiline="false"
          />
          <button
            type="submit"
            className="chat-send-button"
            disabled={isLoading || !inputValue.trim()}
            aria-label="Send message"
          >
            Send
          </button>
        </form>
      </div>
    </div>
  );
};

export default ChatBotPanel;