import { useState, useCallback, useRef } from 'react';
import type {
  Message,
  ChatState,
  ApiStreamChunk,
  Citation,
  ApiCitation,
} from '../types/chat';

const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://physical-ai-book-k7w7.onrender.com'
  : 'http://localhost:8000';

function generateId(): string {
  return `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}

function convertApiCitation(apiCitation: ApiCitation): Citation {
  return {
    chapter: apiCitation.chapter,
    chapterTitle: apiCitation.chapter_title,
    sourceFile: apiCitation.source_file,
    excerpt: apiCitation.excerpt,
  };
}

interface UseStreamingChatReturn {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  sendMessage: (question: string) => Promise<void>;
  clearMessages: () => void;
  clearError: () => void;
}

export function useStreamingChat(): UseStreamingChatReturn {
  const [state, setState] = useState<ChatState>({
    messages: [],
    isLoading: false,
    error: null,
    streamingMessageId: null,
  });

  const abortControllerRef = useRef<AbortController | null>(null);

  const clearMessages = useCallback(() => {
    setState({
      messages: [],
      isLoading: false,
      error: null,
      streamingMessageId: null,
    });
  }, []);

  const clearError = useCallback(() => {
    setState((prev) => ({ ...prev, error: null }));
  }, []);

  const sendMessage = useCallback(async (question: string) => {
    // Cancel any ongoing request
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
    }
    abortControllerRef.current = new AbortController();

    const userMessage: Message = {
      id: generateId(),
      role: 'user',
      content: question,
      timestamp: new Date(),
    };

    const assistantMessageId = generateId();
    const assistantMessage: Message = {
      id: assistantMessageId,
      role: 'assistant',
      content: '',
      timestamp: new Date(),
      isStreaming: true,
    };

    setState((prev) => ({
      ...prev,
      messages: [...prev.messages, userMessage, assistantMessage],
      isLoading: true,
      error: null,
      streamingMessageId: assistantMessageId,
    }));

    try {
      const response = await fetch(`${API_BASE_URL}/api/ask/stream`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question }),
        signal: abortControllerRef.current.signal,
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const reader = response.body?.getReader();
      if (!reader) {
        throw new Error('Response body is not readable');
      }

      const decoder = new TextDecoder();
      let buffer = '';
      let accumulatedContent = '';
      let finalCitations: Citation[] | undefined;
      let finalConfidence: number | undefined;
      let isLowConfidence: boolean | undefined;

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop() || '';

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            const jsonStr = line.slice(6).trim();
            if (!jsonStr) continue;

            try {
              const chunk: ApiStreamChunk = JSON.parse(jsonStr);

              if (chunk.error) {
                setState((prev) => ({
                  ...prev,
                  messages: prev.messages.map((msg) =>
                    msg.id === assistantMessageId
                      ? {
                          ...msg,
                          error: chunk.error || undefined,
                          isStreaming: false,
                        }
                      : msg
                  ),
                  isLoading: false,
                  streamingMessageId: null,
                }));
                return;
              }

              accumulatedContent += chunk.content;

              if (chunk.citations) {
                finalCitations = chunk.citations.map(convertApiCitation);
              }

              if (chunk.confidence !== null && chunk.confidence !== undefined) {
                finalConfidence = chunk.confidence;
              }

              if (chunk.lowConfidence !== undefined) {
                isLowConfidence = chunk.lowConfidence;
              }

              setState((prev) => ({
                ...prev,
                messages: prev.messages.map((msg) =>
                  msg.id === assistantMessageId
                    ? {
                        ...msg,
                        content: accumulatedContent,
                        citations: finalCitations,
                        confidence: finalConfidence,
                        lowConfidence: isLowConfidence,
                        isStreaming: !chunk.done,
                      }
                    : msg
                ),
                isLoading: !chunk.done,
                streamingMessageId: chunk.done ? null : assistantMessageId,
              }));
            } catch {
              // Ignore JSON parse errors for incomplete chunks
            }
          }
        }
      }
    } catch (err) {
      if (err instanceof Error && err.name === 'AbortError') {
        return;
      }

      const errorMessage =
        err instanceof Error ? err.message : 'An unexpected error occurred';

      setState((prev) => ({
        ...prev,
        messages: prev.messages.map((msg) =>
          msg.id === assistantMessageId
            ? {
                ...msg,
                error: errorMessage,
                isStreaming: false,
              }
            : msg
        ),
        isLoading: false,
        error: errorMessage,
        streamingMessageId: null,
      }));
    }
  }, []);

  return {
    messages: state.messages,
    isLoading: state.isLoading,
    error: state.error,
    sendMessage,
    clearMessages,
    clearError,
  };
}
