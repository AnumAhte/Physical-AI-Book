import React, { createContext, useContext, useState, useCallback, useMemo } from 'react';
import { useStreamingChat } from '../../hooks/useStreamingChat';
import type { Message } from '../../types/chat';
import { emitChatAnalytics } from './analytics';

const MAX_MESSAGES = 50;

interface FloatingChatContextType {
  isOpen: boolean;
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  togglePanel: () => void;
  openPanel: () => void;
  closePanel: () => void;
  sendMessage: (question: string) => Promise<void>;
  clearConversation: () => void;
  clearError: () => void;
}

const FloatingChatContext = createContext<FloatingChatContextType | null>(null);

interface FloatingChatProviderProps {
  children: React.ReactNode;
}

export function FloatingChatProvider({ children }: FloatingChatProviderProps): React.ReactElement {
  const [isOpen, setIsOpen] = useState(false);
  const {
    messages: rawMessages,
    isLoading,
    error,
    sendMessage: baseSendMessage,
    clearMessages,
    clearError,
  } = useStreamingChat();

  // Enforce 50-message limit with FIFO trimming (FR-014)
  const messages = useMemo(() => {
    if (rawMessages.length <= MAX_MESSAGES) {
      return rawMessages;
    }
    return rawMessages.slice(-MAX_MESSAGES);
  }, [rawMessages]);

  const openPanel = useCallback(() => {
    setIsOpen(true);
    emitChatAnalytics('chat_opened');
  }, []);

  const closePanel = useCallback(() => {
    setIsOpen(false);
  }, []);

  const togglePanel = useCallback(() => {
    if (isOpen) {
      closePanel();
    } else {
      openPanel();
    }
  }, [isOpen, openPanel, closePanel]);

  const sendMessage = useCallback(async (question: string) => {
    const trimmedQuestion = question.trim();
    if (!trimmedQuestion) return;

    emitChatAnalytics('message_sent', { messageLength: trimmedQuestion.length });

    try {
      await baseSendMessage(trimmedQuestion);
    } catch (err) {
      emitChatAnalytics('error_occurred', {
        error: err instanceof Error ? err.message : 'Unknown error'
      });
      throw err;
    }
  }, [baseSendMessage]);

  const clearConversation = useCallback(() => {
    clearMessages();
  }, [clearMessages]);

  const value = useMemo(() => ({
    isOpen,
    messages,
    isLoading,
    error,
    togglePanel,
    openPanel,
    closePanel,
    sendMessage,
    clearConversation,
    clearError,
  }), [isOpen, messages, isLoading, error, togglePanel, openPanel, closePanel, sendMessage, clearConversation, clearError]);

  return (
    <FloatingChatContext.Provider value={value}>
      {children}
    </FloatingChatContext.Provider>
  );
}

export function useFloatingChat(): FloatingChatContextType {
  const context = useContext(FloatingChatContext);
  if (!context) {
    throw new Error('useFloatingChat must be used within a FloatingChatProvider');
  }
  return context;
}
