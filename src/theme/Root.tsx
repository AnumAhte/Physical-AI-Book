import React from 'react';
import { FloatingChatProvider, FloatingChatBubble } from '../components/FloatingChat';

interface RootProps {
  children: React.ReactNode;
}

// This component wraps the entire Docusaurus app
// It provides the FloatingChat context and renders the bubble globally
export default function Root({ children }: RootProps): React.ReactElement {
  return (
    <FloatingChatProvider>
      {children}
      <FloatingChatBubble />
    </FloatingChatProvider>
  );
}
