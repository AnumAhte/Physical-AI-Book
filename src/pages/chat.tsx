import React from 'react';
import Layout from '@theme/Layout';
import ChatContainer from '../components/Chat/ChatContainer';
import styles from '../css/chat.module.css';

export default function ChatPage(): React.ReactElement {
  return (
    <Layout
      title="Ask the Textbook"
      description="Ask questions about the Physical AI textbook and get AI-powered answers"
    >
      <main className={styles.chatPage}>
        <ChatContainer />
      </main>
    </Layout>
  );
}
