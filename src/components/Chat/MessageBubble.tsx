import React from 'react';
import type { Message, Citation } from '../../types/chat';
import styles from '../../css/chat.module.css';

interface MessageBubbleProps {
  message: Message;
}

function CitationList({ citations }: { citations: Citation[] }): React.ReactElement | null {
  if (citations.length === 0) return null;

  return (
    <div className={styles.citationsContainer}>
      <div className={styles.citationsHeader}>Sources</div>
      <div className={styles.citationsList}>
        {citations.map((citation, index) => (
          <div key={index} className={styles.sourceCard}>
            <div className={styles.sourceTitle}>
              {citation.chapterTitle || citation.chapter}
            </div>
            <div className={styles.sourceExcerpt}>{citation.excerpt}</div>
          </div>
        ))}
      </div>
    </div>
  );
}

function ConfidenceBadge({
  confidence,
}: {
  confidence: number;
}): React.ReactElement | null {
  if (confidence === undefined || confidence === null) return null;

  let level: 'high' | 'medium' | 'low';
  let label: string;

  if (confidence >= 0.7) {
    level = 'high';
    label = 'High confidence';
  } else if (confidence >= 0.4) {
    level = 'medium';
    label = 'Medium confidence';
  } else {
    level = 'low';
    label = 'Low confidence';
  }

  return (
    <span className={`${styles.confidenceBadge} ${styles[level]}`}>
      {label} ({Math.round(confidence * 100)}%)
    </span>
  );
}

function LowConfidenceWarning(): React.ReactElement {
  return (
    <div className={styles.lowConfidenceWarning}>
      <span className={styles.warningIcon}>⚠️</span>
      <span>This question may be outside the textbook's scope. The response may not be reliable.</span>
    </div>
  );
}

export default function MessageBubble({
  message,
}: MessageBubbleProps): React.ReactElement {
  const isUser = message.role === 'user';
  const isError = !!message.error;
  // Use explicit flag from backend, or fallback to confidence threshold
  const isLowConfidence = message.lowConfidence ??
    (message.confidence !== undefined && message.confidence < 0.4);

  const bubbleClasses = [
    styles.messageBubble,
    styles[message.role],
    isError && styles.error,
    isLowConfidence && styles.lowConfidence,
  ]
    .filter(Boolean)
    .join(' ');

  return (
    <div className={`${styles.messageWrapper} ${styles[message.role]}`}>
      <div className={`${styles.avatar} ${styles[message.role]}`}>
        {isUser ? '👤' : '🤖'}
      </div>
      <div className={bubbleClasses}>
        {/* Show warning banner for low confidence responses */}
        {!isUser && isLowConfidence && !message.isStreaming && (
          <LowConfidenceWarning />
        )}

        <div className={styles.messageContent}>
          {message.error ? (
            <span style={{ color: 'var(--ifm-color-danger)' }}>
              {message.error}
            </span>
          ) : (
            message.content
          )}
          {message.isStreaming && <span className={styles.cursor}>▋</span>}
        </div>

        {!isUser && message.confidence !== undefined && !message.isStreaming && (
          <ConfidenceBadge confidence={message.confidence} />
        )}

        {!isUser &&
          message.citations &&
          message.citations.length > 0 &&
          !message.isStreaming && <CitationList citations={message.citations} />}
      </div>
    </div>
  );
}
