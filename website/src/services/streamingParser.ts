/**
 * Streaming Response Parser for RAG Chat
 *
 * Parses JSON-lines stream from /api/chat-stream endpoint.
 * Handles different chunk types (text, citations, metadata, errors).
 */

export interface TextChunk {
  type: 'text';
  content: string;
  index: number;
}

export interface CitationChunk {
  type: 'citations';
  citations: Array<{
    chunk_id: string;
    chapter_name: string;
    section_name?: string;
    source_url: string;
    page_num?: number;
    score?: number;
  }>;
}

export interface MetadataChunk {
  type: 'metadata';
  latency_ms: number;
  model: string;
  mode: string;
  request_id?: string;
}

export interface ErrorChunk {
  type: 'error';
  error: string;
  message: string;
  request_id?: string;
}

export type StreamChunk = TextChunk | CitationChunk | MetadataChunk | ErrorChunk;

/**
 * Parser for streaming responses
 */
export class StreamingResponseParser {
  private buffer: string = '';
  private textChunks: string[] = [];
  private citations: CitationChunk['citations'] = [];
  private metadata: MetadataChunk | null = null;
  private error: ErrorChunk | null = null;

  /**
   * Parse a chunk of data from the stream
   *
   * @param data Incoming data chunk
   * @param onChunk Callback for each parsed JSON chunk
   */
  parseChunk(data: string, onChunk: (chunk: StreamChunk) => void): void {
    this.buffer += data;
    const lines = this.buffer.split('\n');

    // Keep the last incomplete line in buffer
    this.buffer = lines.pop() || '';

    for (const line of lines) {
      if (!line.trim()) continue;

      try {
        const chunk = JSON.parse(line) as StreamChunk;

        // Track different chunk types
        switch (chunk.type) {
          case 'citations':
            this.citations = chunk.citations;
            break;
          case 'text':
            this.textChunks.push(chunk.content);
            break;
          case 'metadata':
            this.metadata = chunk;
            break;
          case 'error':
            this.error = chunk;
            break;
        }

        onChunk(chunk);
      } catch (parseError) {
        console.error('Failed to parse JSON line:', line, parseError);
      }
    }
  }

  /**
   * Finalize parsing and return accumulated data
   *
   * @returns Accumulated response data
   */
  finalize(): {
    text: string;
    citations: CitationChunk['citations'];
    metadata: MetadataChunk | null;
    error: ErrorChunk | null;
  } {
    // Parse any remaining buffer
    if (this.buffer.trim()) {
      try {
        const chunk = JSON.parse(this.buffer) as StreamChunk;
        switch (chunk.type) {
          case 'citations':
            this.citations = chunk.citations;
            break;
          case 'text':
            this.textChunks.push(chunk.content);
            break;
          case 'metadata':
            this.metadata = chunk;
            break;
          case 'error':
            this.error = chunk;
            break;
        }
      } catch (parseError) {
        console.error('Failed to parse final buffer:', this.buffer);
      }
    }

    return {
      text: this.textChunks.join(''),
      citations: this.citations,
      metadata: this.metadata,
      error: this.error,
    };
  }

  /**
   * Reset parser state
   */
  reset(): void {
    this.buffer = '';
    this.textChunks = [];
    this.citations = [];
    this.metadata = null;
    this.error = null;
  }

  /**
   * Get accumulated text so far
   */
  getAccumulatedText(): string {
    return this.textChunks.join('');
  }

  /**
   * Get accumulated citations so far
   */
  getAccumulatedCitations() {
    return this.citations;
  }
}
