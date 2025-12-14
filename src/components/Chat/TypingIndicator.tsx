import React from 'react';
import styles from '../../css/chat.module.css';

export default function TypingIndicator(): React.ReactElement {
  return (
    <div className={`${styles.messageWrapper} ${styles.assistant}`}>
      <div className={`${styles.avatar} ${styles.assistant}`}>
        🤖
      </div>
      <div className={`${styles.messageBubble} ${styles.assistant}`}>
        <div className={styles.typingIndicator}>
          <span className={styles.typingDot} />
          <span className={styles.typingDot} />
          <span className={styles.typingDot} />
        </div>
      </div>
    </div>
  );
}
