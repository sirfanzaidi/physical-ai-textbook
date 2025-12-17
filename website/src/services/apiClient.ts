/**
 * API Client for RAG Backend
 *
 * Provides async methods for interacting with the RAG backend API.
 * Handles authentication, error handling, and request/response formatting.
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import { ragConfig } from '../config/ragConfig';

interface ChatRequest {
  query: string;
  book_id?: string;
  mode?: 'full' | 'selected';
  selected_text?: string;
  top_k?: number;
}

interface Citation {
  chunk_id: string;
  chapter_name: string;
  section_name?: string;
  source_url: string;
  page_num?: number;
}

interface ChatResponse {
  response_text: string;
  citations: Citation[];
  sources: string[];
  latency_ms: number;
  model: string;
}

interface ErrorResponse {
  error: string;
  message: string;
  status_code: number;
}

class APIClient {
  private client: AxiosInstance;

  constructor(baseURL: string) {
    this.client = axios.create({
      baseURL,
      headers: {
        'Content-Type': 'application/json',
      },
      timeout: 60000, // 60s timeout for streaming responses
    });

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error: AxiosError) => {
        console.error('API Error:', error.message);
        throw error;
      }
    );
  }

  /**
   * Stream chat response from RAG backend
   *
   * @param request Chat request
   * @param onChunk Callback for each streamed chunk
   * @param onError Callback for errors
   * @returns Promise that resolves when stream completes
   */
  async chatStream(
    request: ChatRequest,
    onChunk: (chunk: any) => void,
    onError: (error: Error) => void
  ): Promise<void> {
    try {
      const response = await this.client.post('/api/chat-stream', request, {
        responseType: 'stream',
      });

      const reader = response.data;

      // For browser environments, use ReadableStream
      if (reader instanceof ReadableStream) {
        const textDecoder = new TextDecoder();
        const buffer = reader.getReader();

        try {
          while (true) {
            const { done, value } = await buffer.read();
            if (done) break;

            const text = textDecoder.decode(value, { stream: true });
            const lines = text.split('\n').filter((line) => line.trim());

            for (const line of lines) {
              try {
                const chunk = JSON.parse(line);
                onChunk(chunk);
              } catch (parseError) {
                console.error('Failed to parse chunk:', line);
              }
            }
          }
        } finally {
          buffer.releaseLock();
        }
      } else {
        // Node.js environment (for testing)
        // This shouldn't happen in browser, but handle it gracefully
        const textDecoder = new TextDecoder();
        let buffer = '';

        reader.on('data', (chunk: Buffer) => {
          buffer += textDecoder.decode(chunk, { stream: true });
          const lines = buffer.split('\n');

          // Keep the last incomplete line in the buffer
          buffer = lines.pop() || '';

          for (const line of lines) {
            if (line.trim()) {
              try {
                const parsed = JSON.parse(line);
                onChunk(parsed);
              } catch (parseError) {
                console.error('Failed to parse chunk:', line);
              }
            }
          }
        });

        reader.on('end', () => {
          if (buffer.trim()) {
            try {
              const parsed = JSON.parse(buffer);
              onChunk(parsed);
            } catch (parseError) {
              console.error('Failed to parse final chunk:', buffer);
            }
          }
        });

        reader.on('error', (error: Error) => {
          onError(error);
        });
      }
    } catch (error) {
      if (error instanceof AxiosError) {
        const errorMsg = (error.response?.data as ErrorResponse)?.message || error.message;
        onError(new Error(errorMsg));
      } else {
        onError(error instanceof Error ? error : new Error(String(error)));
      }
    }
  }

  /**
   * Non-streaming chat endpoint (for fallback)
   *
   * @param request Chat request
   * @returns Chat response
   */
  async chat(request: ChatRequest): Promise<ChatResponse> {
    try {
      const response = await this.client.post<ChatResponse>('/api/chat', request);
      return response.data;
    } catch (error) {
      if (error instanceof AxiosError) {
        const errorMsg = (error.response?.data as ErrorResponse)?.message || error.message;
        throw new Error(errorMsg);
      }
      throw error;
    }
  }

  /**
   * Get ingestion status
   *
   * @returns Collection status
   */
  async getIngestStatus(): Promise<any> {
    try {
      const response = await this.client.get('/api/ingest/status');
      return response.data;
    } catch (error) {
      console.error('Failed to get ingest status:', error);
      return { status: 'unknown' };
    }
  }

  /**
   * Health check
   *
   * @returns Health status
   */
  async healthCheck(): Promise<any> {
    try {
      const response = await this.client.get('/api/health');
      return response.data;
    } catch (error) {
      console.error('Health check failed:', error);
      return { status: 'unhealthy' };
    }
  }
}

// Initialize with base URL from config
export const apiClient = new APIClient(ragConfig.apiBaseURL);

export type { ChatRequest, Citation, ChatResponse, ErrorResponse };
