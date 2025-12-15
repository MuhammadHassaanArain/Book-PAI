// Storage utility for chat data using localStorage

import { ChatSession, ChatUIState } from './types';

const CHAT_STORAGE_KEY = 'chatState';
const SESSIONS_STORAGE_KEY = 'chatSessions';

export interface StorageState {
  session: ChatSession | null;
  uiState: ChatUIState;
  lastSessionId: string | null;
}

export const loadChatState = (): StorageState => {
  try {
    const savedState = localStorage.getItem(CHAT_STORAGE_KEY);
    if (savedState) {
      const parsedState = JSON.parse(savedState);

      // Convert string dates back to Date objects
      if (parsedState.session) {
        parsedState.session.createdAt = new Date(parsedState.session.createdAt);
        parsedState.session.lastActive = new Date(parsedState.session.lastActive);

        // Convert message timestamps too
        if (parsedState.session.messages) {
          parsedState.session.messages = parsedState.session.messages.map((msg: any) => ({
            ...msg,
            timestamp: new Date(msg.timestamp)
          }));
        }
      }

      return parsedState;
    }
  } catch (error) {
    console.error('Failed to load chat state from localStorage', error);
  }

  return {
    session: null,
    uiState: {
      isPanelOpen: false,
      isMinimized: false,
      unreadCount: 0,
    },
    lastSessionId: null,
  };
};

export const saveChatState = (state: StorageState): void => {
  try {
    // Convert Date objects to strings for JSON serialization
    const stateToSave = {
      ...state,
      session: state.session ? {
        ...state.session,
        createdAt: state.session.createdAt.toISOString(),
        lastActive: state.session.lastActive.toISOString(),
        messages: state.session.messages.map(msg => ({
          ...msg,
          timestamp: msg.timestamp.toISOString()
        }))
      } : null
    };

    localStorage.setItem(CHAT_STORAGE_KEY, JSON.stringify(stateToSave));
  } catch (error) {
    console.error('Failed to save chat state to localStorage', error);
  }
};

export const clearChatState = (): void => {
  try {
    localStorage.removeItem(CHAT_STORAGE_KEY);
  } catch (error) {
    console.error('Failed to clear chat state from localStorage', error);
  }
};