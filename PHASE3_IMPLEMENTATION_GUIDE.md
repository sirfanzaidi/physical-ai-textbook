# Phase 3 Implementation Guide: Query Book Content Anywhere (User Story 1)

## Current Status

✅ **Backend Complete (Phase 1-2)**
- FastAPI application with CORS configured
- OpenRouter client for embeddings and chat
- Qdrant vector store integration
- RAG chatbot orchestration layer
- Chat streaming endpoint (`POST /api/chat-stream`)
- Ingestion service (models + extraction + chunking)

## Phase 3: User Story 1 - Query Book Content Anywhere (MVP Core)

### Goal
Users can ask questions about book content from any Docusaurus page via a floating widget and receive answers with source citations.

### Frontend Components Needed

#### 1. **ChatWindow.tsx**
- Render conversation message list
- Display citations with links to source pages
- Show loading state
- Auto-scroll to latest message
- Display metadata (latency, model used)

#### 2. **InputBox.tsx**
- Query text input field
- Character counter (max 5000)
- Send button (disabled while loading)
- Keyboard shortcut support (Ctrl+Enter)
- Placeholder text from config

#### 3. **FloatingButton.tsx**
- Floating action button (bottom-right corner)
- Icon (chat bubble or message icon)
- Opens/closes chat modal
- Optional badge for unread messages
- Responsive for mobile

#### 4. **Citation.tsx**
- Render individual citation
- Show chapter/section info
- Clickable link to source page
- Similarity score badge
- Preview text truncation

#### 5. **RAGChatContainer.tsx** (Main Component)
- Combine ChatWindow + InputBox + FloatingButton
- Manage chat state (messages, loading, errors)
- Handle API calls to FastAPI backend
- Error boundary and user-friendly error messages
- Keyboard event handling

#### 6. **useRAGChat.ts** (Custom Hook)
- Chat state management (messages, pending, error)
- Query history tracking
- Selected text management (for US2)
- API call orchestration
- Streaming response parsing

#### 7. **apiClient.ts**
- Axios wrapper for API calls
- Base URL configuration from ragConfig
- Error handling and retry logic
- Request/response interceptors
- Streaming response handler (JSON-lines parsing)

#### 8. **streamingClient.ts**
- Parse JSON-lines streaming responses
- Accumulate text chunks
- Extract citations from stream
- Handle connection errors
- Timeout after 30s

### API Integration

**Endpoint**: `POST /api/chat-stream`

**Request**:
```json
{
  "query": "What is physical AI?",
  "mode": "full",
  "book_id": "physical-ai",
  "top_k": 5
}
```

**Response** (JSON-lines):
```json
{"type":"citations","citations":[...]}
{"type":"text","content":"Physical AI is..","index":0}
{"type":"text","content":"...","index":1}
{"type":"metadata","latency_ms":1234,"model":"qwen/qwen3-14b-chat","mode":"full"}
```

### Integration with Docusaurus

#### Option A: Theme Component (Recommended)
- Add RAGChat to `src/theme/Layout/index.tsx`
- Renders globally on all pages
- Clean integration with Docusaurus styling

#### Option B: Swizzle
- Swizzle `DocSidebar` or `Footer` component
- Inject RAGChat widget
- More control over placement

### Testing Checklist

- [ ] Floating button appears on all pages
- [ ] Click opens chat modal
- [ ] Can type query and submit
- [ ] Response appears within 5 seconds
- [ ] Citations display with links
- [ ] Modal can be closed
- [ ] Multiple queries work in sequence
- [ ] Error messages are user-friendly
- [ ] Works on mobile (responsive)
- [ ] Keyboard shortcuts work (if implemented)

### Files to Create

```
website/src/
├── components/
│   └── RAGChat/
│       ├── index.tsx (main container)
│       ├── ChatWindow.tsx
│       ├── InputBox.tsx
│       ├── FloatingButton.tsx
│       ├── Citation.tsx
│       ├── RAGChat.module.css (styles)
│       └── types.ts (TypeScript interfaces)
├── hooks/
│   └── useRAGChat.ts
├── services/
│   ├── apiClient.ts
│   └── streamingClient.ts
├── config/
│   └── ragConfig.ts (update existing)
└── theme/
    └── Layout/
        └── index.tsx (add RAGChat import)
```

### Configuration

**ragConfig.ts** should export:
- `apiBaseURL`: Backend URL (from .env or auto-detect)
- `bookId`: Book identifier (default: "physical-ai")
- `maxQueryLength`: 5000 characters
- `minSelectedTextLength`: 10 characters (for US2)
- `topK`: Default context chunks to retrieve (5)

### Success Metrics (User Story 1)

✅ **Functional**:
- Users can ask questions from any page
- Response contains relevant content
- Citations link to source chapters
- Latency < 5 seconds

✅ **Quality**:
- Accuracy >= 90% (answers match book content)
- No hallucinations (answers grounded in retrieved context)
- Clear error messages for API failures

✅ **UX**:
- Widget doesn't break Docusaurus layout
- Mobile responsive
- Accessibility compliant (ARIA labels, keyboard nav)

## Next Steps

1. Create `useRAGChat.ts` hook with state management
2. Create `apiClient.ts` and `streamingClient.ts` services
3. Build `RAGChatContainer.tsx` and child components
4. Style components (responsive design, dark mode)
5. Integrate into Docusaurus theme
6. Test with sample queries
7. Deploy to Vercel

## Dependencies

Ensure `website/package.json` includes:
- `axios`: HTTP client
- `typescript`: Type safety
- Any UI icon library (e.g., `lucide-react` for icons)

## Environment Setup

Create `.env.local` in website root:
```env
REACT_APP_RAG_API_BASE_URL=http://localhost:8000
REACT_APP_BOOK_ID=physical-ai
```

For production (Vercel):
```env
REACT_APP_RAG_API_BASE_URL=https://your-api-domain.com
REACT_APP_BOOK_ID=physical-ai
```
