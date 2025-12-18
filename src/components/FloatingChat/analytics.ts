export type ChatAnalyticsEvent = 'chat_opened' | 'message_sent' | 'error_occurred';

interface AnalyticsEventDetail {
  event: ChatAnalyticsEvent;
  timestamp: number;
  data?: Record<string, unknown>;
}

/**
 * Emit analytics events for chat interactions (FR-015)
 * Uses CustomEvent for loose coupling - can be integrated with any analytics provider
 */
export function emitChatAnalytics(
  event: ChatAnalyticsEvent,
  data?: Record<string, unknown>
): void {
  const detail: AnalyticsEventDetail = {
    event,
    timestamp: Date.now(),
    data,
  };

  // Dispatch custom event that can be captured by analytics integrations
  if (typeof window !== 'undefined') {
    window.dispatchEvent(
      new CustomEvent('chat_analytics', { detail })
    );

    // Also log to console in development for debugging
    if (process.env.NODE_ENV === 'development') {
      console.log('[ChatAnalytics]', event, data || '');
    }
  }
}
