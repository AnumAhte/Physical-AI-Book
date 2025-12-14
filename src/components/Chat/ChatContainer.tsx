import React from 'react';
import { useStreamingChat } from '../../hooks/useStreamingChat';
import ChatInput from './ChatInput';
import MessageList from './MessageList';
import ErrorMessage from './ErrorMessage';
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
        <ErrorMessage message={error} onDismiss={clearError} />
      )}

      <ChatInput onSend={handleSend} disabled={isLoading} />
    </div>
  );
}
