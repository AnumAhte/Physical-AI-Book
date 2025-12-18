# Quickstart: Floating Chat Bubble

**Feature**: 002-floating-chat-bubble
**Date**: 2025-12-17

## Prerequisites

- Node.js 18+ installed
- Project cloned and dependencies installed (`npm install`)
- Backend RAG service running (or accessible at https://ai-book-rag.onrender.com)

## Development Setup

### 1. Start Development Server

```bash
npm start
```

Opens at http://localhost:3000

### 2. Verify Existing Chat Works

Navigate to http://localhost:3000/chat and confirm:
- Chat input accepts messages
- Streaming responses display
- No console errors

This validates the backend connection and existing chat infrastructure.

## Implementation Steps

### Step 1: Swizzle Root Component

```bash
npx docusaurus swizzle @docusaurus/theme-classic Root --wrap --typescript
```

This creates `src/theme/Root.tsx` which wraps the entire app.

### Step 2: Create FloatingChat Components

Create the following files:

```
src/components/FloatingChat/
├── FloatingChatContext.tsx  # State provider
├── FloatingChatBubble.tsx   # Trigger button
├── FloatingChatPanel.tsx    # Expanded chat UI
└── index.ts                 # Exports
```

### Step 3: Create Floating Chat Styles

```
src/css/floatingChat.module.css
```

### Step 4: Wire Up Root Wrapper

Edit `src/theme/Root.tsx` to wrap children with `FloatingChatProvider` and render `FloatingChatBubble`.

### Step 5: Test Across Pages

1. Open any documentation page
2. Click the floating bubble (bottom-right)
3. Send a message
4. Navigate to another page
5. Verify conversation persists
6. Press Esc to close panel
7. Test on mobile viewport (320px width)

## Key Files Reference

| File | Purpose |
|------|---------|
| `src/theme/Root.tsx` | Global wrapper, injects floating chat |
| `src/components/FloatingChat/FloatingChatContext.tsx` | Cross-page state management |
| `src/components/FloatingChat/FloatingChatBubble.tsx` | Floating button (collapsed state) |
| `src/components/FloatingChat/FloatingChatPanel.tsx` | Chat panel (expanded state) |
| `src/css/floatingChat.module.css` | Floating-specific styles |
| `src/hooks/useStreamingChat.ts` | Existing streaming logic (reused) |
| `src/components/Chat/*` | Existing chat components (reused) |

## Testing Checklist

### Functional Tests

- [ ] Bubble visible on all doc pages
- [ ] Click bubble opens panel
- [ ] Send message receives streaming response
- [ ] Close button/Esc closes panel
- [ ] Conversation persists across navigation
- [ ] "New Chat" clears conversation
- [ ] Error state displays on backend failure

### Keyboard Accessibility

- [ ] Tab focuses bubble, then panel elements
- [ ] Enter submits message
- [ ] Esc closes panel from anywhere

### Mobile Responsive

- [ ] Bubble visible at 320px width
- [ ] Panel expands to appropriate size
- [ ] Touch targets minimum 44px
- [ ] Input doesn't trigger unwanted zoom

### Analytics Events

- [ ] `chat_opened` fires on panel open
- [ ] `message_sent` fires on send
- [ ] `error_occurred` fires on error

## Troubleshooting

### Bubble Not Appearing

1. Check `src/theme/Root.tsx` exists and exports correctly
2. Verify `FloatingChatBubble` is rendered in Root
3. Check browser console for errors
4. Verify CSS is being loaded (check z-index)

### State Not Persisting

1. Ensure `FloatingChatProvider` wraps entire app in Root.tsx
2. Check for multiple provider instances
3. Verify not using hard navigation (should be client-side)

### Backend Connection Failed

1. Check network tab for /api/ask/stream requests
2. Verify BASE_URL in useStreamingChat matches environment
3. Test backend directly: `curl https://ai-book-rag.onrender.com/health`

## Build Verification

```bash
npm run build
npm run serve
```

Test the production build locally before deployment.
