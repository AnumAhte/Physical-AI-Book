# Research: Floating Chat Bubble

**Feature**: 002-floating-chat-bubble
**Date**: 2025-12-17
**Status**: Complete

## Research Tasks

### 1. Docusaurus Theme Swizzling for Global Components

**Decision**: Use `@theme/Root` wrapper (wrap mode) to inject FloatingChatProvider globally.

**Rationale**:
- Root.tsx wraps the entire Docusaurus app, including the router
- React Context placed here persists across page navigations (SPA behavior)
- "Wrap" mode is safer than "eject" - preserves upgrade compatibility
- No need to modify individual pages or layouts

**Alternatives Considered**:
- Layout swizzle: Would require changes to every layout variant
- Client module: Less control over component tree placement
- Plugin: Overkill for a simple UI component

**Implementation**:
```bash
# Creates src/theme/Root.tsx
npx docusaurus swizzle @docusaurus/theme-classic Root --wrap
```

### 2. Cross-Page State Persistence Pattern

**Decision**: React Context with useStreamingChat hook integration.

**Rationale**:
- Docusaurus is a SPA - context state persists during client-side navigation
- No external state library needed (keeps bundle small)
- useStreamingChat hook already handles all chat logic; context just lifts state up
- 50-message limit easily enforced with array slicing

**Alternatives Considered**:
- sessionStorage: Would require serialization, lose streaming state
- Redux/Zustand: Adds dependencies, complexity for simple state
- URL state: Not appropriate for conversation data

**Pattern**:
```typescript
// FloatingChatContext wraps useStreamingChat
// Adds: isOpen state, togglePanel, message limit enforcement
// Exposes same interface as useStreamingChat + panel controls
```

### 3. Floating UI Positioning Best Practices

**Decision**: CSS fixed positioning with responsive adjustments.

**Rationale**:
- `position: fixed` keeps bubble visible during scroll
- Bottom-right is conventional for chat widgets (user expectation)
- High z-index (9999) ensures visibility over Docusaurus UI
- CSS transforms for animations (GPU-accelerated)

**Alternatives Considered**:
- Floating-ui library: Overkill, adds dependency
- Absolute positioning: Would scroll with content
- Portal to body: Unnecessary with fixed positioning

**Implementation Details**:
```css
.floatingBubble {
  position: fixed;
  bottom: 24px;
  right: 24px;
  z-index: 9999;
}

.floatingPanel {
  position: fixed;
  bottom: 80px;  /* Above bubble */
  right: 24px;
  width: 380px;
  max-height: 500px;
  z-index: 9998;
}

/* Mobile: Full width panel */
@media (max-width: 480px) {
  .floatingPanel {
    width: calc(100% - 32px);
    right: 16px;
    bottom: 72px;
    max-height: 60vh;
  }
}
```

### 4. Keyboard Accessibility Implementation

**Decision**: Focus trap in panel, Esc to close, Tab navigation.

**Rationale**:
- WCAG 2.1 compliance requires keyboard operability
- Esc to close is universal pattern for modals/overlays
- Focus trap prevents tabbing out of panel accidentally
- Enter to submit already exists in ChatInput

**Implementation**:
```typescript
// FloatingChatPanel.tsx
useEffect(() => {
  const handleKeyDown = (e: KeyboardEvent) => {
    if (e.key === 'Escape') {
      closePanel();
    }
  };
  if (isOpen) {
    document.addEventListener('keydown', handleKeyDown);
    // Focus input on open
    inputRef.current?.focus();
  }
  return () => document.removeEventListener('keydown', handleKeyDown);
}, [isOpen]);
```

### 5. Analytics Event Pattern

**Decision**: CustomEvent dispatch to window for loose coupling.

**Rationale**:
- No analytics library currently in project
- CustomEvent allows future integration with any analytics provider
- Decouples chat component from analytics implementation
- Easy to add Google Analytics, Plausible, etc. later

**Alternatives Considered**:
- Direct GA calls: Couples to specific provider
- Context callback: Adds complexity to context interface
- Console logging: Not production-ready

**Implementation**:
```typescript
// Analytics utility
export function emitChatAnalytics(event: 'chat_opened' | 'message_sent' | 'error_occurred', data?: object) {
  window.dispatchEvent(new CustomEvent('chat_analytics', {
    detail: { event, timestamp: Date.now(), ...data }
  }));
}

// Usage
emitChatAnalytics('chat_opened');
emitChatAnalytics('message_sent', { messageLength: question.length });
emitChatAnalytics('error_occurred', { error: errorMessage });
```

### 6. Animation Performance

**Decision**: CSS transitions with transform/opacity only.

**Rationale**:
- transform and opacity are GPU-accelerated (no layout thrash)
- 300ms duration matches spec requirement (SC-004)
- Cubic-bezier easing for natural feel
- No JavaScript animation libraries needed

**Implementation**:
```css
.floatingPanel {
  transform: translateY(20px);
  opacity: 0;
  transition: transform 300ms cubic-bezier(0.4, 0, 0.2, 1),
              opacity 300ms cubic-bezier(0.4, 0, 0.2, 1);
}

.floatingPanel.open {
  transform: translateY(0);
  opacity: 1;
}
```

## Resolved Clarifications

All technical unknowns resolved. No NEEDS CLARIFICATION items remain.

| Topic | Resolution |
|-------|------------|
| Global injection method | Docusaurus Root swizzle (wrap mode) |
| State persistence | React Context above router |
| Positioning approach | CSS fixed positioning |
| Keyboard handling | useEffect with keydown listener |
| Analytics pattern | CustomEvent dispatch |
| Animation approach | CSS transitions (transform/opacity) |

## References

- [Docusaurus Swizzling Guide](https://docusaurus.io/docs/swizzling)
- [React Context Best Practices](https://react.dev/reference/react/useContext)
- [WCAG 2.1 Keyboard Guidelines](https://www.w3.org/WAI/WCAG21/Understanding/keyboard)
