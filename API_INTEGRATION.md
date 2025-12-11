# Frontend API Integration Guide

Guide for integrating the FastAPI backend with the Docusaurus frontend.

## API Endpoints

### Health Check
```
GET /api/health

Response:
{
  "status": "healthy",
  "qdrant_connected": true,
  "collection_exists": true,
  "collection_count": 6,
  "cohere_available": true
}
```

### Chat Endpoint
```
POST /api/chat

Request:
{
  "question": "What is physical AI?",
  "selected_text": null,
  "chapter_filter": null
}

Response:
{
  "answer": "Physical AI is...",
  "sources": [
    {
      "chapter_num": 1,
      "chapter_title": "Introduction",
      "chapter_url": "https://...",
      "snippet": "..."
    }
  ],
  "retrieved_chunks_count": 3
}
```

## Docusaurus Plugin Example

Create a chatbot plugin in the website:

```typescript
// website/plugins/chatbot-plugin.ts

export default function chatbotPlugin(context, options) {
  return {
    name: 'chatbot-plugin',
    async loadContent() {
      // Initialize connection to backend
      return {
        apiUrl: process.env.RAG_API_URL || 'http://localhost:8000'
      };
    },
    
    async contentLoaded({content, actions}) {
      const {setGlobalData} = actions;
      setGlobalData({apiUrl: content.apiUrl});
    }
  };
}
```

## React Hook for Chat

```typescript
// website/src/hooks/useChat.ts

import {useState} from 'react';

export interface ChatMessage {
  question: string;
  answer: string;
  sources: Source[];
  timestamp: Date;
}

export interface Source {
  chapter_num: number;
  chapter_title: string;
  chapter_url: string;
  snippet: string;
}

export function useChat(apiUrl: string) {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const sendMessage = async (question: string) => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch(`${apiUrl}/api/chat`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({question})
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();
      
      setMessages(prev => [...prev, {
        question,
        answer: data.answer,
        sources: data.sources,
        timestamp: new Date()
      }]);

    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
    } finally {
      setLoading(false);
    }
  };

  return {messages, loading, error, sendMessage};
}
```

## React Component Example

```typescript
// website/src/components/ChatbotWidget.tsx

import React, {useRef, useEffect} from 'react';
import {useChat} from '../hooks/useChat';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function ChatbotWidget() {
  const {siteConfig} = useDocusaurusContext();
  const apiUrl = (siteConfig as any).customFields?.ragApiUrl || 'http://localhost:8000';
  
  const {messages, loading, error, sendMessage} = useChat(apiUrl);
  const [input, setInput] = React.useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({behavior: 'smooth'});
  };

  useEffect(scrollToBottom, [messages]);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim()) return;
    
    sendMessage(input);
    setInput('');
  };

  return (
    <div className="chatbot-widget">
      <div className="messages">
        {messages.map((msg, idx) => (
          <div key={idx} className="message-pair">
            <div className="question">Q: {msg.question}</div>
            <div className="answer">A: {msg.answer}</div>
            {msg.sources.length > 0 && (
              <div className="sources">
                <strong>Sources:</strong>
                <ul>
                  {msg.sources.map(src => (
                    <li key={src.chapter_num}>
                      <a href={src.chapter_url}>
                        {src.chapter_title}
                      </a>
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </div>
        ))}
        {loading && <div className="loading">Thinking...</div>}
        {error && <div className="error">Error: {error}</div>}
        <div ref={messagesEndRef}/>
      </div>

      <form onSubmit={handleSubmit} className="chat-form">
        <input
          type="text"
          value={input}
          onChange={e => setInput(e.target.value)}
          placeholder="Ask about the textbook..."
          disabled={loading}
        />
        <button type="submit" disabled={loading}>
          {loading ? 'Thinking...' : 'Send'}
        </button>
      </form>
    </div>
  );
}
```

## Docusaurus Config

Update `docusaurus.config.ts`:

```typescript
const config: Config = {
  // ... existing config ...
  
  customFields: {
    ragApiUrl: process.env.RAG_API_URL || 'http://localhost:8000'
  },

  plugins: [
    // ... existing plugins ...
    require.resolve('./plugins/chatbot-plugin.ts')
  ]
};
```

## Environment Setup

Create `.env.local` in website:

```
RAG_API_URL=https://your-deployed-api.example.com
```

## Testing

1. Start backend:
```bash
cd backend
python -m uvicorn api.app:app --reload
```

2. Start website:
```bash
cd website
npm run start
```

3. Test chatbot widget at http://localhost:3000

## Deployment

### For Local Testing
No changes needed - frontend will use `http://localhost:8000`

### For Production
Update `vercel.json`:
```json
{
  "env": {
    "RAG_API_URL": "@rag-api-url"
  }
}
```

Add to Vercel environment variables:
```
RAG_API_URL=https://your-railway-api.up.railway.app
```

## Error Handling

The hook handles:
- Network errors
- API timeouts
- Invalid responses
- Missing environment variables

All errors are returned in the `error` state.

## Performance Tips

1. **Caching**: Cache frequently asked questions
2. **Debouncing**: Debounce search inputs
3. **Lazy Loading**: Load chatbot widget on demand
4. **Compression**: Enable gzip for responses

## CORS Configuration

API is already configured for CORS:
```
Access-Control-Allow-Origins: https://physical-ai-textbook-two.vercel.app
Access-Control-Allow-Methods: GET, POST, OPTIONS
```

Update if deploying to different domain.
