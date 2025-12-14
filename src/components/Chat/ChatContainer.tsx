import React from 'react';
import { useStreamingChat } from '../../hooks/useStreamingChat';
import ChatInput from './ChatInput';
import MessageList from './MessageList';
import styles from '../../css/chat.module.css';

export default function ChatContainer(): React.ReactElement {
  const { messages, isLoading, error, sendMessage, clearError } =
    useStreamingChat();

  const handleSend = async (question: string) => {
    clearError();
    await sendMessage(question);
  };

  const showTypingIndicator =
    isLoading &&
    messages.length > 0 &&
    !messages[messages.length - 1]?.isStreaming;

  return (
    <div className={styles.chatContainer}>
      <div className={styles.chatHeader}>
        <h1>Ask the Textbook</h1>
        <p>Get answers from the Physical AI textbook content</p>
      </div>

      <MessageList
        messages={messages}
        showTypingIndicator={showTypingIndicator}
      />

      {error && (
        <div className={styles.errorContainer}>
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="currentColor"
            width="20"
            height="20"
          >
            <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm1 15h-2v-2h2v2zm0-4h-2V7h2v6z" />
          </svg>
          <span>{error}</span>
        </div>
      )}

      <ChatInput onSend={handleSend} disabled={isLoading} />
    </div>
  );
}
