import React, { useRef, useEffect } from 'react';
import type { Message } from '../../types/chat';
import MessageBubble from './MessageBubble';
import TypingIndicator from './TypingIndicator';
import styles from '../../css/chat.module.css';

interface MessageListProps {
  messages: Message[];
  showTypingIndicator: boolean;
}

function EmptyState(): React.ReactElement {
  return (
    <div className={styles.emptyState}>
      <svg
        xmlns="http://www.w3.org/2000/svg"
        viewBox="0 0 24 24"
        fill="currentColor"
      >
        <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" />
      </svg>
      <p>Ask a question about the textbook...</p>
    </div>
  );
}

export default function MessageList({
  messages,
  showTypingIndicator,
}: MessageListProps): React.ReactElement {
  const containerRef = useRef<HTMLDivElement>(null);
  const bottomRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive or content updates
  useEffect(() => {
    if (bottomRef.current) {
      bottomRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages]);

  // Also scroll when streaming content updates
  useEffect(() => {
    const lastMessage = messages[messages.length - 1];
    if (lastMessage?.isStreaming && bottomRef.current) {
      bottomRef.current.scrollIntoView({ behavior: 'auto' });
    }
  }, [messages]);

  return (
    <div ref={containerRef} className={styles.messageList}>
      {messages.length === 0 ? (
        <EmptyState />
      ) : (
        <>
          {messages.map((message) => (
            <MessageBubble key={message.id} message={message} />
          ))}
          {showTypingIndicator && <TypingIndicator />}
          {/* Invisible element to scroll to */}
          <div ref={bottomRef} />
        </>
      )}
    </div>
  );
}
