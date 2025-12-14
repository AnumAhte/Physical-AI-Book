/**
 * TypeScript types for RAG Chatbot
 */

export interface Citation {
  chapter: string;
  chapterTitle: string;
  sourceFile: string;
  excerpt: string;
}

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  confidence?: number;
  lowConfidence?: boolean;
  timestamp: Date;
  isStreaming?: boolean;
  error?: string;
}

export interface ChatState {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  streamingMessageId: string | null;
}

export interface StreamEvent {
  content: string;
  done: boolean;
  citations?: Citation[];
  confidence?: number;
  error?: string;
}

export interface AskRequest {
  question: string;
  top_k?: number;
}

// API response types (snake_case from backend)
export interface ApiCitation {
  chapter: string;
  chapter_title: string;
  source_file: string;
  excerpt: string;
}

export interface ApiStreamChunk {
  content: string;
  done: boolean;
  citations?: ApiCitation[] | null;
  confidence?: number | null;
  lowConfidence?: boolean;
  error?: string | null;
}

// Helper to convert API citation to frontend format
export function convertCitation(apiCitation: ApiCitation): Citation {
  return {
    chapter: apiCitation.chapter,
    chapterTitle: apiCitation.chapter_title,
    sourceFile: apiCitation.source_file,
    excerpt: apiCitation.excerpt,
  };
}
