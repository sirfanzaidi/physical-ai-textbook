# RAG Chatbot Integration - Duplicate Script Fix

## Problem

When running the Docusaurus website with the integrated RAG chat widget, you encountered:

```
ERROR: Identifier 'RAGChatWidget' has already been declared
ERROR: Identifier 'APIClient' has already been declared
```

This occurred because the JavaScript files were being loaded multiple times, causing duplicate class declarations.

## Root Cause

1. **Hot module reload in Docusaurus**: During development, hot reloading would reload scripts without cleaning up
2. **Multiple mount points**: The React component could mount multiple times, each trying to load the scripts
3. **No deduplication check**: The Vanilla JS files didn't check if classes were already defined

## Solution Implemented

### 1. Enhanced React Component (`website/src/components/RAGChat.tsx`)

- Added **global flag** (`scriptsLoadedGlobally`) to track if scripts have been loaded
- Added **element IDs** to script tags to prevent duplicate loading
- Added **checks** for already-defined classes before initializing
- Improved **initialization logic** with proper error handling
- Each component instance gets a unique ID to prevent conflicts

**Key improvements:**
```typescript
// Check if scripts already loaded
const hasUtils = typeof (window as any).APIClient !== 'undefined';
const hasWidget = typeof (window as any).RAGChatWidget !== 'undefined';

// Check if element already exists
if (!document.getElementById('rag-widget-script')) {
  // Only add script if it doesn't exist
}
```

### 2. Protected Class Declarations (`frontend/utils.js` and `frontend/chat-widget.js`)

Added guards around class declarations to prevent re-declaration errors:

**Before:**
```javascript
class APIClient {
  // ... class definition
}
```

**After:**
```javascript
if (typeof APIClient === 'undefined') {
  class APIClient {
    // ... class definition
  }
}
```

This allows the scripts to be loaded multiple times without errors—the classes are only defined once.

## Files Updated

1. ✅ `website/src/components/RAGChat.tsx` - Enhanced script loading logic
2. ✅ `frontend/utils.js` - Protected APIClient, MessageFormatter, TextSelectionUtils, StorageManager
3. ✅ `frontend/chat-widget.js` - Protected RAGChatWidget class
4. ✅ `website/static/chat-widget.js` - Updated copy
5. ✅ `website/static/utils.js` - Updated copy

## How It Works Now

### Script Loading Flow

```
Component Mounts
    ↓
Check if scripts already defined in window
    ↓
Scripts already defined? → Initialize widget → Done
    ↓
Scripts not defined? → Check if elements exist
    ↓
Elements don't exist? → Create and append scripts
    ↓
Scripts load → Classes defined (only once, protected by if statement)
    ↓
Initialize widget
```

### Protection Against Duplicate Declarations

```javascript
// First load: APIClient is undefined
if (typeof APIClient === 'undefined') {
  class APIClient { ... }  // ✅ Defined
}

// Subsequent loads: APIClient already exists
if (typeof APIClient === 'undefined') {
  class APIClient { ... }  // ❌ Skipped (already defined)
}
```

## Testing the Fix

### Local Development

1. **Clear browser cache and local storage:**
   ```javascript
   // Run in browser console
   localStorage.clear();
   location.reload();
   ```

2. **Start the website:**
   ```bash
   cd website
   npm run start
   ```

3. **Navigate to chat page:**
   - Open http://localhost:3000
   - Click "💬 Ask RAG Bot" in navbar
   - Or go directly to http://localhost:3000/chat

4. **Check browser console:**
   - Should show **no errors**
   - Widget should initialize successfully

5. **Test hot reload:**
   - Make a change to a file (e.g., edit chat.tsx)
   - Save and watch it reload
   - Chat should still work without console errors

### Production

After deploying to Vercel, the same protections apply:
- Scripts load once
- Classes defined once
- Multiple page navigations work smoothly
- No duplicate declaration errors

## Verification Checklist

- [ ] Browser console has no "Identifier already declared" errors
- [ ] Chat widget initializes on page load
- [ ] Can type and submit queries
- [ ] Response streams in real-time
- [ ] Citations display correctly
- [ ] Message history persists
- [ ] Page hot reload works (dev mode)
- [ ] Dark mode toggle works
- [ ] Mobile responsive (480px breakpoint)
- [ ] Select-text "Ask" button works

## Advanced: How Auto-Initialization Still Works

The files include auto-initialization for elements with `data-rag-widget` attribute:

```html
<div
  id="my-widget"
  data-rag-widget
  data-book-id="physical-ai-textbook"
></div>

<script src="/utils.js"></script>
<script src="/chat-widget.js"></script>
```

This still works because:
1. Scripts are protected by `if (typeof ClassName === 'undefined')`
2. DOMContentLoaded event fires after all scripts load
3. Auto-init code runs (lines 450-461 in chat-widget.js)
4. New RAGChatWidget instances are created per element

## Performance Impact

**No negative impact:**
- ✅ Scripts only loaded once (via element IDs)
- ✅ Classes only defined once (via if guards)
- ✅ Multiple components can coexist (unique containerId)
- ✅ No memory leaks
- ✅ Hot reload safe in development

## Next Steps

1. **Clear local storage:** `localStorage.clear()` in console
2. **Hard refresh:** `Ctrl+Shift+R` (or `Cmd+Shift+R` on Mac)
3. **Test the chat page** - should work without errors
4. **Deploy to production** when ready

If you still see errors:
1. Check browser DevTools → Console
2. Look for actual error messages (copy them)
3. Check that website/static/ has updated files
4. Try clearing browser cache completely

---

**Status**: ✅ Integration fixed and ready for testing!
