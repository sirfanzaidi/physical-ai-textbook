/**
 * Simplified useRAGChat Hook
 *
 * Basic hook for managing RAG chat state and API communication
 */
import { useState, useCallback } from 'react';
import { apiClient } from '../services/apiClient';

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: number;
}

interface RAGChatState {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  selectedText: string | null;
  mode: 'full' | 'selected';
}

const generateMessageId = (): string => {
  return `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
};

export function useRAGChat() {
  const [state, setState] = useState<RAGChatState>({
    messages: [],
    isLoading: false,
    error: null,
    selectedText: null,
    mode: 'full',
  });

  const sendQuery = useCallback(async (query: string) => {
    if (!query.trim()) return;

    // Add user message immediately
    const userMessage: Message = {
      id: generateMessageId(),
      role: 'user',
      content: query,
      timestamp: Date.now(),
    };

    setState(prev => ({
      ...prev,
      messages: [...prev.messages, userMessage],
      isLoading: true,
      error: null,
    }));

    try {
      // Call the backend API
      const requestPayload: any = {
        query,
        book_id: 'physical-ai-textbook',
        mode: state.mode,
        top_k: 5,
      };

      // Add selected_text if in selected mode
      if (state.mode === 'selected' && state.selectedText) {
        requestPayload.selected_text = state.selectedText;
      }

      const response = await apiClient.chat(requestPayload);

      // Add assistant response
      const assistantMessage: Message = {
        id: generateMessageId(),
        role: 'assistant',
        content: response.response_text,
        timestamp: Date.now(),
      };

      setState(prev => ({
        ...prev,
        messages: [...prev.messages, assistantMessage],
        isLoading: false,
        error: null,
      }));
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error occurred';

      setState(prev => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
      }));
    }
  }, [state.mode, state.selectedText]);

  const setSelectedText = useCallback((text: string | null) => {
    setState(prev => ({
      ...prev,
      selectedText: text,
      mode: text ? 'selected' : 'full',
    }));
  }, []);

  const clearHistory = useCallback(() => {
    setState(prev => ({
      ...prev,
      messages: [],
      isLoading: false,
      error: null,
      selectedText: null,
      mode: 'full',
    }));
  }, []);

  return {
    messages: state.messages,
    isLoading: state.isLoading,
    error: state.error,
    selectedText: state.selectedText,
    mode: state.mode,
    sendQuery,
    setSelectedText,
    clearHistory,
  };
}