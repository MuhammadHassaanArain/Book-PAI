import React, { createContext, useContext, useReducer, useEffect } from 'react';
import { ChatMessage, ChatSession, ChatUIState, ChatState, ChatContextType } from './types';
import { loadChatState, saveChatState } from './storage';

// Initial state
const initialState: ChatState = {
  session: null,
  uiState: {
    isPanelOpen: false,
    isMinimized: false,
    unreadCount: 0,
  },
};

// Action types
type ChatAction =
  | { type: 'ADD_MESSAGE'; payload: ChatMessage }
  | { type: 'CLEAR_MESSAGES' }
  | { type: 'TOGGLE_PANEL' }
  | { type: 'UPDATE_UI_STATE'; payload: Partial<ChatUIState> }
  | { type: 'SET_SESSION'; payload: ChatSession };

// Reducer
const chatReducer = (state: ChatState, action: ChatAction): ChatState => {
  switch (action.type) {
    case 'ADD_MESSAGE':
      if (!state.session) {
        // Create a new session if none exists
        const newSession: ChatSession = {
          id: `session_${Date.now()}`,
          messages: [action.payload],
          createdAt: new Date(),
          lastActive: new Date(),
          isActive: true,
        };
        return { ...state, session: newSession };
      }

      return {
        ...state,
        session: {
          ...state.session,
          messages: [...state.session.messages, action.payload],
          lastActive: new Date(),
        },
      };
    case 'CLEAR_MESSAGES':
      return {
        ...state,
        session: state.session
          ? { ...state.session, messages: [] }
          : null,
      };
    case 'TOGGLE_PANEL':
      return {
        ...state,
        uiState: {
          ...state.uiState,
          isPanelOpen: !state.uiState.isPanelOpen,
        },
      };
    case 'UPDATE_UI_STATE':
      return {
        ...state,
        uiState: { ...state.uiState, ...action.payload },
      };
    case 'SET_SESSION':
      return {
        ...state,
        session: action.payload,
      };
    default:
      return state;
  }
};

// Create context
const ChatContext = createContext<ChatContextType | undefined>(undefined);

// Provider component
export const ChatProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [state, dispatch] = useReducer(chatReducer, undefined , () => {
    // Load initial state from storage
    const loadedState = loadChatState();
    return {
      session: loadedState.session,
      uiState: loadedState.uiState,
    };
  });

  // Save state to localStorage whenever it changes
  useEffect(() => {
    saveChatState({
      session: state.session,
      uiState: state.uiState,
      lastSessionId: state.session?.id || null,
    });
  }, [state.session, state.uiState]);

  const addMessage = (message: Omit<ChatMessage, 'id' | 'timestamp'>) => {
    const newMessage: ChatMessage = {
      ...message,
      id: `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date(),
    };
    dispatch({ type: 'ADD_MESSAGE', payload: newMessage });
  };

  const clearMessages = () => {
    dispatch({ type: 'CLEAR_MESSAGES' });
  };

  const togglePanel = () => {
    dispatch({ type: 'TOGGLE_PANEL' });
  };

  const updateUIState = (newState: Partial<ChatUIState>) => {
    dispatch({ type: 'UPDATE_UI_STATE', payload: newState });
  };

  return (
    <ChatContext.Provider
      value={{
        ...state,
        addMessage,
        clearMessages,
        togglePanel,
        updateUIState,
      }}
    >
      {children}
    </ChatContext.Provider>
  );
};

// Custom hook to use the chat context
export const useChat = (): ChatContextType => {
  const context = useContext(ChatContext);
  if (context === undefined) {
    throw new Error('useChat must be used within a ChatProvider');
  }
  return context;
};