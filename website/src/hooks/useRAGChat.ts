/**
 * useRAGChat Hook
 *
 * Custom React hook for managing RAG chat state and interactions.
 * Handles message history, streaming, errors, and API communication.
 */

import { useState, useCallback, useRef } from 'react';
import { apiClient, ChatRequest, Citation, ChatResponse } from '../services/apiClient';
import { StreamingResponseParser, StreamChunk } from '../services/streamingParser';
import { ragConfig } from '../config/ragConfig';

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  metadata?: {
    latency_ms?: number;
    model?: string;
  };
  timestamp: number;
}

export interface RAGChatState {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  selectedText: string | null;
  mode: 'full' | 'selected';
}

const generateMessageId = (): string => {
  return `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
};

/**
 * Custom hook for RAG chat functionality
 *
 * @returns Chat state and control methods
 */
export function useRAGChat() {
  const [state, setState] = useState<RAGChatState>({
    messages: [],
    isLoading: false,
    error: null,
    selectedText: null,
    mode: 'full',
  });

  const parserRef = useRef<StreamingResponseParser>(new StreamingResponseParser());
  const abortControllerRef = useRef<AbortController | null>(null);

  /**
   * Send a query to the RAG backend
   *
   * @param query User query
   * @param useStreaming Whether to use streaming response
   */
  const sendQuery = useCallback(
    async (query: string, useStreaming: boolean = true) => {
      if (!query.trim()) {
        setState((prev) => ({ ...prev, error: 'Query cannot be empty' }));
        return;
      }

      // Validate query length
      if (query.length > ragConfig.maxQueryLength) {
        setState((prev) => ({
          ...prev,
          error: `Query exceeds maximum length of ${ragConfig.maxQueryLength} characters`,
        }));
        return;
      }

      // Add user message
      const userMessage: Message = {
        id: generateMessageId(),
        role: 'user',
        content: query,
        timestamp: Date.now(),
      };

      setState((prev) => ({
        ...prev,
        messages: [...prev.messages, userMessage],
        isLoading: true,
        error: null,
      }));

      try {
        // Prepare request
        const request: ChatRequest = {
          query,
          book_id: ragConfig.bookId,
          mode: state.mode,
          selected_text: state.selectedText || undefined,
          top_k: 5,
        };

        if (useStreaming) {
          // Use streaming endpoint
          await handleStreamingQuery(request);
        } else {
          // Use regular endpoint
          await handleNonStreamingQuery(request);
        }
      } catch (err) {
        const errorMessage = err instanceof Error ? err.message : 'Unknown error occurred';
        setState((prev) => ({
          ...prev,
          isLoading: false,
          error: errorMessage,
        }));
      }
    },
    [state.mode, state.selectedText]
  );

  /**
   * Handle streaming query response
   */
  const handleStreamingQuery = async (request: ChatRequest) => {
    const parser = parserRef.current;
    parser.reset();

    const assistantMessageId = generateMessageId();
    let assistantContent = '';
    let citations: Citation[] = [];

    // Create abort controller for this request
    abortControllerRef.current = new AbortController();

    const onChunk = (chunk: StreamChunk) => {
      switch (chunk.type) {
        case 'text':
          assistantContent += chunk.content;
          // Update message as text arrives
          setState((prev) => {
            const messages = [...prev.messages];
            const lastMessage = messages[messages.length - 1];
            if (lastMessage && lastMessage.id === assistantMessageId) {
              lastMessage.content = assistantContent;
            }
            return { ...prev, messages };
          });
          break;

        case 'citations':
          citations = chunk.citations.map((c) => ({
            chunk_id: c.chunk_id,
            chapter_name: c.chapter_name,
            section_name: c.section_name,
            source_url: c.source_url,
            page_num: c.page_num,
          }));
          break;

        case 'metadata':
          setState((prev) => {
            const messages = [...prev.messages];
            const lastMessage = messages[messages.length - 1];
            if (lastMessage && lastMessage.id === assistantMessageId) {
              lastMessage.metadata = {
                latency_ms: chunk.latency_ms,
                model: chunk.model,
              };
            }
            return { ...prev, messages };
          });
          break;

        case 'error':
          throw new Error(chunk.message);
      }
    };

    const onError = (error: Error) => {
      setState((prev) => ({
        ...prev,
        isLoading: false,
        error: error.message,
      }));
    };

    // Start streaming
    await apiClient.chatStream(request, onChunk, onError);

    // Finalize and add complete assistant message
    const final = parser.finalize();

    const assistantMessage: Message = {
      id: assistantMessageId,
      role: 'assistant',
      content: final.text,
      citations: final.citations.length > 0 ? final.citations : undefined,
      metadata: final.metadata ? {
        latency_ms: final.metadata.latency_ms,
        model: final.metadata.model,
      } : undefined,
      timestamp: Date.now(),
    };

    setState((prev) => ({
      ...prev,
      messages: [...prev.messages, assistantMessage],
      isLoading: false,
      error: null,
    }));
  };

  /**
   * Handle non-streaming query response
   */
  const handleNonStreamingQuery = async (request: ChatRequest) => {
    const response = await apiClient.chat(request);

    const assistantMessage: Message = {
      id: generateMessageId(),
      role: 'assistant',
      content: response.response_text,
      citations: response.citations.length > 0 ? response.citations : undefined,
      metadata: {
        latency_ms: response.latency_ms,
        model: response.model,
      },
      timestamp: Date.now(),
    };

    setState((prev) => ({
      ...prev,
      messages: [...prev.messages, assistantMessage],
      isLoading: false,
      error: null,
    }));
  };

  /**
   * Set selected text (for US2: Ask about selected text)
   */
  const setSelectedText = useCallback((text: string | null) => {
    setState((prev) => ({
      ...prev,
      selectedText: text,
      mode: text ? 'selected' : 'full',
    }));
  }, []);

  /**
   * Clear chat history
   */
  const clearHistory = useCallback(() => {
    setState((prev) => ({
      ...prev,
      messages: [],
      error: null,
      selectedText: null,
      mode: 'full',
    }));
  }, []);

  /**
   * Cancel current operation
   */
  const cancel = useCallback(() => {
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
      abortControllerRef.current = null;
    }
    setState((prev) => ({
      ...prev,
      isLoading: false,
    }));
  }, []);

  return {
    // State
    messages: state.messages,
    isLoading: state.isLoading,
    error: state.error,
    selectedText: state.selectedText,
    mode: state.mode,

    // Methods
    sendQuery,
    setSelectedText,
    clearHistory,
    cancel,
  };
}
