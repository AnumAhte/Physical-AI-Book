import React from 'react';
import { useFloatingChat } from './FloatingChatContext';
import FloatingChatPanel from './FloatingChatPanel';
import styles from '../../css/floatingChat.module.css';

// Chat bubble SVG icon
function ChatIcon(): React.ReactElement {
  return (
    <svg
      className={styles.bubbleIcon}
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
    </svg>
  );
}

export default function FloatingChatBubble(): React.ReactElement {
  const { isOpen, togglePanel } = useFloatingChat();

  return (
    <>
      {/* Floating bubble button */}
      <button
        type="button"
        className={`${styles.floatingBubble} ${isOpen ? styles.hidden : ''}`}
        onClick={togglePanel}
        aria-label="Open chat assistant"
        aria-expanded={isOpen}
        aria-haspopup="dialog"
      >
        <ChatIcon />
      </button>

      {/* Chat panel */}
      <FloatingChatPanel />
    </>
  );
}
