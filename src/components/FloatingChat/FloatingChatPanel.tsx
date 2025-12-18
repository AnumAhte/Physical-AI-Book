import React, { useEffect, useRef } from 'react';
import { useFloatingChat } from './FloatingChatContext';
import MessageList from '../Chat/MessageList';
import ChatInput from '../Chat/ChatInput';
import ErrorMessage from '../Chat/ErrorMessage';
import styles from '../../css/floatingChat.module.css';

// Close icon SVG
function CloseIcon(): React.ReactElement {
  return (
    <svg
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <line x1="18" y1="6" x2="6" y2="18" />
      <line x1="6" y1="6" x2="18" y2="18" />
    </svg>
  );
}

// Refresh/New chat icon SVG
function RefreshIcon(): React.ReactElement {
  return (
    <svg
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <path d="M3 12a9 9 0 0 1 9-9 9.75 9.75 0 0 1 6.74 2.74L21 8" />
      <path d="M21 3v5h-5" />
      <path d="M21 12a9 9 0 0 1-9 9 9.75 9.75 0 0 1-6.74-2.74L3 16" />
      <path d="M3 21v-5h5" />
    </svg>
  );
}

// Empty state for floating panel
function FloatingEmptyState(): React.ReactElement {
  return (
    <div className={styles.emptyState}>
      <svg
        viewBox="0 0 24 24"
        fill="currentColor"
        aria-hidden="true"
      >
        <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" />
      </svg>
      <p>Ask about the textbook...</p>
    </div>
  );
}

export default function FloatingChatPanel(): React.ReactElement {
  const {
    isOpen,
    messages,
    isLoading,
    error,
    closePanel,
    sendMessage,
    clearConversation,
    clearError,
  } = useFloatingChat();

  const panelRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Handle Esc key to close panel (FR-013)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        closePanel();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, closePanel]);

  // Auto-focus input when panel opens (FR-013)
  useEffect(() => {
    if (isOpen) {
      // Small delay to allow animation to start
      const timer = setTimeout(() => {
        const textarea = panelRef.current?.querySelector('textarea');
        textarea?.focus();
      }, 100);
      return () => clearTimeout(timer);
    }
  }, [isOpen]);

  const handleSend = async (question: string) => {
    await sendMessage(question);
  };

  return (
    <div
      ref={panelRef}
      className={`${styles.floatingPanel} ${isOpen ? styles.open : ''}`}
      role="dialog"
      aria-modal="true"
      aria-label="Chat assistant"
      aria-hidden={!isOpen}
    >
      {/* Header */}
      <div className={styles.panelHeader}>
        <h2 className={styles.panelTitle}>Ask the Textbook</h2>
        <div className={styles.headerActions}>
          {messages.length > 0 && (
            <button
              type="button"
              className={`${styles.headerButton} ${styles.newChatButton}`}
              onClick={clearConversation}
              aria-label="Start new conversation"
              title="New chat"
            >
              <RefreshIcon />
              <span>New</span>
            </button>
          )}
          <button
            type="button"
            className={styles.headerButton}
            onClick={closePanel}
            aria-label="Close chat panel"
            title="Close"
          >
            <CloseIcon />
          </button>
        </div>
      </div>

      {/* Content - Message Area */}
      <div className={styles.panelContent}>
        {error && (
          <ErrorMessage message={error} onDismiss={clearError} />
        )}
        {messages.length === 0 ? (
          <FloatingEmptyState />
        ) : (
          <MessageList
            messages={messages}
            showTypingIndicator={isLoading && !messages[messages.length - 1]?.isStreaming}
          />
        )}
      </div>

      {/* Footer - Input Area */}
      <div className={styles.panelFooter}>
        <ChatInput
          onSend={handleSend}
          disabled={isLoading}
          placeholder="Ask a question..."
        />
      </div>
    </div>
  );
}
