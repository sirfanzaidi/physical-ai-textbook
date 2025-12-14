/**
 * Utility functions for RAG chatbot frontend.
 *
 * Provides API helpers, message formatting, and response parsing.
 */

class APIClient {
  /**
   * Initialize API client with base URL.
   *
   * @param {string} baseURL - Base URL for API (default: window.location.origin)
   */
  constructor(baseURL = null) {
    this.baseURL = baseURL || window.location.origin;
    this.timeout = 30000; // 30 second timeout
  }

  /**
   * Make API request with error handling.
   *
   * @param {string} method - HTTP method (GET, POST, etc.)
   * @param {string} endpoint - API endpoint path
   * @param {object} data - Request payload (for POST/PUT)
   * @returns {Promise<object>} Response data
   */
  async request(method, endpoint, data = null) {
    const url = `${this.baseURL}/api${endpoint}`;
    const options = {
      method,
      headers: {
        'Content-Type': 'application/json',
      },
    };

    if (data) {
      options.body = JSON.stringify(data);
    }

    try {
      const response = await Promise.race([
        fetch(url, options),
        new Promise((_, reject) =>
          setTimeout(() => reject(new Error('Request timeout')), this.timeout)
        ),
      ]);

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.detail || `HTTP ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error(`API request failed: ${error.message}`);
      throw error;
    }
  }

  /**
   * Send chat query.
   *
   * @param {object} query - Query object {query, book_id, mode, selected_text}
   * @returns {Promise<object>} Chat response
   */
  async chat(query) {
    return this.request('POST', '/chat', query);
  }

  /**
   * Upload book file for ingestion.
   *
   * @param {File} file - File to upload
   * @param {string} bookId - Optional book ID
   * @returns {Promise<object>} Ingestion response
   */
  async ingestBook(file, bookId = null) {
    const formData = new FormData();
    formData.append('file', file);
    if (bookId) {
      formData.append('book_id', bookId);
    }

    const url = `${this.baseURL}/api/ingest`;
    const options = {
      method: 'POST',
      body: formData,
      // Note: Don't set Content-Type header - browser will set it with boundary
    };

    try {
      const response = await fetch(url, options);
      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.detail || `HTTP ${response.status}`);
      }
      return await response.json();
    } catch (error) {
      console.error(`Book ingestion failed: ${error.message}`);
      throw error;
    }
  }

  /**
   * Stream chat response.
   *
   * @param {object} query - Query object
   * @param {function} onChunk - Callback for each streamed chunk
   * @returns {Promise<void>}
   */
  async chatStream(query, onChunk) {
    const url = `${this.baseURL}/api/chat-stream`;
    const options = {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(query),
    };

    try {
      const response = await fetch(url, options);
      if (!response.ok) throw new Error(`HTTP ${response.status}`);

      const reader = response.body.getReader();
      const decoder = new TextDecoder();

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        const chunk = decoder.decode(value, { stream: true });
        // Parse JSON lines
        chunk.split('\n').forEach((line) => {
          if (line.trim()) {
            try {
              const data = JSON.parse(line);
              onChunk(data);
            } catch (e) {
              console.warn('Failed to parse stream chunk:', line);
            }
          }
        });
      }
    } catch (error) {
      console.error(`Stream failed: ${error.message}`);
      throw error;
    }
  }
}

/**
 * Message formatting utilities.
 */
class MessageFormatter {
  /**
   * Format message for display.
   *
   * @param {string} text - Message text
   * @param {string} role - 'user' or 'assistant'
   * @returns {object} Formatted message
   */
  static formatMessage(text, role) {
    return {
      text: text.trim(),
      role,
      timestamp: new Date().toISOString(),
      id: `msg_${Date.now()}_${Math.random()}`,
    };
  }

  /**
   * Parse citations from response.
   *
   * @param {object} response - Chat response object
   * @returns {array} Formatted citations
   */
  static parseCitations(response) {
    if (!response.citations || !Array.isArray(response.citations)) {
      return [];
    }

    return response.citations.map((citation, index) => ({
      index: index + 1,
      pageNum: citation.page_num,
      sectionName: citation.section_name,
      chapterName: citation.chapter_name,
      snippet: citation.text_snippet,
    }));
  }

  /**
   * Format response with citations.
   *
   * @param {object} response - Chat response
   * @returns {string} Formatted response text
   */
  static formatResponseWithCitations(response) {
    let text = response.response || '';

    if (response.citations && response.citations.length > 0) {
      text += '\n\n**Sources:**\n';
      response.citations.forEach((citation, index) => {
        const parts = [];
        if (citation.chapter_name) parts.push(citation.chapter_name);
        if (citation.page_num) parts.push(`p. ${citation.page_num}`);
        const source = parts.length > 0 ? ` (${parts.join(', ')})` : '';
        text += `[${index + 1}]${source}\n`;
      });
    }

    return text;
  }

  /**
   * Format confidence score as percentage.
   *
   * @param {number} confidence - Confidence (0-1)
   * @returns {string} Formatted confidence
   */
  static formatConfidence(confidence) {
    const percent = Math.round((confidence || 0) * 100);
    return `${percent}%`;
  }

  /**
   * Format latency as human-readable.
   *
   * @param {number} latencyMs - Latency in milliseconds
   * @returns {string} Formatted latency
   */
  static formatLatency(latencyMs) {
    if (latencyMs < 1000) return `${Math.round(latencyMs)}ms`;
    return `${(latencyMs / 1000).toFixed(1)}s`;
  }
}

/**
 * Text selection utilities.
 */
class TextSelectionUtils {
  /**
   * Get currently selected text.
   *
   * @returns {string|null} Selected text or null if nothing selected
   */
  static getSelectedText() {
    const selected = window.getSelection().toString();
    return selected && selected.trim() ? selected.trim() : null;
  }

  /**
   * Check if text is selected.
   *
   * @returns {boolean} True if text is selected
   */
  static hasSelection() {
    return window.getSelection().toString().length > 0;
  }

  /**
   * Get selection coordinates for button placement.
   *
   * @returns {object|null} {x, y} coordinates or null
   */
  static getSelectionCoordinates() {
    const selection = window.getSelection();
    if (selection.rangeCount === 0) return null;

    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    return {
      x: rect.right + 10,
      y: rect.top - 10,
    };
  }

  /**
   * Validate selected text length.
   *
   * @param {string} text - Text to validate
   * @param {number} minLength - Minimum characters (default: 10)
   * @param {number} maxLength - Maximum characters (default: 5000)
   * @returns {object} {valid, error}
   */
  static validateSelectedText(text, minLength = 10, maxLength = 5000) {
    if (!text) {
      return { valid: false, error: 'No text selected' };
    }
    if (text.length < minLength) {
      return { valid: false, error: `Selection too short (min ${minLength} chars)` };
    }
    if (text.length > maxLength) {
      return { valid: false, error: `Selection too long (max ${maxLength} chars)` };
    }
    return { valid: true };
  }
}

/**
 * Local storage utilities for message history.
 */
class StorageManager {
  /**
   * Save messages to localStorage.
   *
   * @param {string} bookId - Book identifier
   * @param {array} messages - Message array
   */
  static saveMessages(bookId, messages) {
    try {
      const key = `rag_messages_${bookId}`;
      localStorage.setItem(key, JSON.stringify(messages));
    } catch (error) {
      console.warn('Failed to save messages:', error);
    }
  }

  /**
   * Load messages from localStorage.
   *
   * @param {string} bookId - Book identifier
   * @returns {array} Messages array or empty array
   */
  static loadMessages(bookId) {
    try {
      const key = `rag_messages_${bookId}`;
      const data = localStorage.getItem(key);
      return data ? JSON.parse(data) : [];
    } catch (error) {
      console.warn('Failed to load messages:', error);
      return [];
    }
  }

  /**
   * Clear messages for a book.
   *
   * @param {string} bookId - Book identifier
   */
  static clearMessages(bookId) {
    try {
      const key = `rag_messages_${bookId}`;
      localStorage.removeItem(key);
    } catch (error) {
      console.warn('Failed to clear messages:', error);
    }
  }

  /**
   * Save last selected mode.
   *
   * @param {string} mode - 'full' or 'selected'
   */
  static saveLastMode(mode) {
    try {
      localStorage.setItem('rag_last_mode', mode);
    } catch (error) {
      console.warn('Failed to save mode:', error);
    }
  }

  /**
   * Get last selected mode.
   *
   * @returns {string} Last mode or 'full' default
   */
  static getLastMode() {
    try {
      return localStorage.getItem('rag_last_mode') || 'full';
    } catch {
      return 'full';
    }
  }
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = { APIClient, MessageFormatter, TextSelectionUtils, StorageManager };
}
