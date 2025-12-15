// TypeScript interfaces based on the data model

export interface ChatMessage {
  id: string;
  text: string;
  sender: 'user' | 'bot' | 'system';
  timestamp: Date;
  status?: 'sent' | 'delivered' | 'error';
}

export interface ChatSession {
  id: string;
  messages: ChatMessage[];
  createdAt: Date;
  lastActive: Date;
  isActive: boolean;
}

export interface ChatUIState {
  isPanelOpen: boolean;
  isMinimized: boolean;
  position?: { x: number; y: number };
  unreadCount: number;
}

// Update the ChatMessage interface in ChatContext to use this shared type
export interface ChatState {
  session: ChatSession | null;
  uiState: ChatUIState;
}

export interface ChatContextType extends ChatState {
  addMessage: (message: Omit<ChatMessage, 'id' | 'timestamp'>) => void;
  clearMessages: () => void;
  togglePanel: () => void;
  updateUIState: (newState: Partial<ChatUIState>) => void;
}