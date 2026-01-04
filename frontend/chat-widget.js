/**
 * RAG Chatbot Widget - Vanilla JavaScript component.
 *
 * Embeddable chat widget for book querying with:
 * - Full-book RAG queries
 * - Select-text zero-leakage mode
 * - Message history with localStorage persistence
 * - Streaming response support
 * - Text selection detection with "Ask about this" button
 */

// Check if already defined to prevent duplicate declarations
if (typeof window.RAGChatWidget === 'undefined') {
  class RAGChatWidget {
  /**
   * Initialize chat widget.
   *
   * @param {object} config - Configuration object
   *   - bookId: (required) Book identifier to query
   *   - containerId: (default: 'rag-widget') Container element ID
   *   - apiBaseURL: (default: window.location.origin) API base URL
   *   - enableStreaming: (default: true) Enable streaming responses
   *   - enableHistory: (default: true) Enable message history persistence
   */
  constructor(config = {}) {
    this.bookId = config.bookId;
    this.containerId = config.containerId || 'rag-widget';
    this.enableStreaming = config.enableStreaming !== false;
    this.enableHistory = config.enableHistory !== false;

    if (!this.bookId) {
      throw new Error('bookId is required');
    }

    this.apiClient = new APIClient(config.apiBaseURL);
    this.messages = this.enableHistory ? StorageManager.loadMessages(this.bookId) : [];
    this.selectedText = null;
    this.isLoading = false;
    this.currentMode = this.enableHistory ? StorageManager.getLastMode() : 'full';

    this.init();
  }

  /**
   * Initialize widget DOM and event listeners.
   */
  init() {
    this.container = document.getElementById(this.containerId);
    if (!this.container) {
      throw new Error(`Container ${this.containerId} not found`);
    }

    this.renderWidget();
    this.attachEventListeners();
    this.renderMessages();

    // Listen for text selection in page
    if (document.body) {
      document.addEventListener('mouseup', () => this.checkTextSelection());
      document.addEventListener('touchend', () => this.checkTextSelection());
    }
  }

  /**
   * Render widget HTML structure.
   */
  renderWidget() {
    this.container.innerHTML = `
      <div class="rag-widget">
        <div class="rag-widget__header">
          <h2 class="rag-widget__title">Ask about this book</h2>
          <div class="rag-widget__mode-toggle">
            <label class="rag-widget__mode-label">
              <input
                type="radio"
                name="mode"
                value="full"
                ${this.currentMode === 'full' ? 'checked' : ''}
                class="rag-widget__mode-radio"
              >
              Full Book
            </label>
            <label class="rag-widget__mode-label">
              <input
                type="radio"
                name="mode"
                value="selected"
                ${this.currentMode === 'selected' ? 'checked' : ''}
                class="rag-widget__mode-radio"
              >
              Selected Text
            </label>
          </div>
        </div>

        <div class="rag-widget__messages" id="rag-messages"></div>

        <div class="rag-widget__input-area">
          <textarea
            id="rag-query-input"
            class="rag-widget__input"
            placeholder="Ask a question..."
            rows="3"
          ></textarea>
          <button id="rag-send-btn" class="rag-widget__button rag-widget__button--primary">
            Send
          </button>
          <button
            id="rag-clear-btn"
            class="rag-widget__button rag-widget__button--secondary"
            title="Clear message history"
          >
            Clear
          </button>
        </div>

        <div class="rag-widget__status" id="rag-status"></div>

        <div class="rag-widget__ask-button" id="rag-ask-button" style="display: none;">
          <button class="rag-widget__ask-btn">Ask about this</button>
        </div>
      </div>
    `;

    this.messagesContainer = document.getElementById('rag-messages');
    this.queryInput = document.getElementById('rag-query-input');
    this.sendBtn = document.getElementById('rag-send-btn');
    this.clearBtn = document.getElementById('rag-clear-btn');
    this.statusDiv = document.getElementById('rag-status');
    this.askButton = document.getElementById('rag-ask-button');
    this.askBtn = this.askButton.querySelector('.rag-widget__ask-btn');
  }

  /**
   * Attach event listeners to widget elements.
   */
  attachEventListeners() {
    // Send button click
    this.sendBtn.addEventListener('click', () => this.sendQuery());

    // Enter key to send (Shift+Enter for newline)
    this.queryInput.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        this.sendQuery();
      }
    });

    // Clear button
    this.clearBtn.addEventListener('click', () => this.clearHistory());

    // Mode toggle
    document.querySelectorAll('.rag-widget__mode-radio').forEach((radio) => {
      radio.addEventListener('change', (e) => {
        this.currentMode = e.target.value;
        StorageManager.saveLastMode(this.currentMode);
        this.updateModeUI();
      });
    });

    // Ask button (for selected text)
    this.askBtn.addEventListener('click', () => this.askAboutSelected());
  }

  /**
   * Check if text is selected on page.
   */
  checkTextSelection() {
    const selected = TextSelectionUtils.getSelectedText();

    if (selected && selected.length > 0) {
      this.selectedText = selected;
      this.showAskButton();
    } else {
      this.selectedText = null;
      this.hideAskButton();
    }
  }

  /**
   * Show "Ask about this" button near selected text.
   */
  showAskButton() {
    const coords = TextSelectionUtils.getSelectionCoordinates();
    if (!coords) return;

    this.askButton.style.display = 'block';
    this.askButton.style.left = `${coords.x}px`;
    this.askButton.style.top = `${coords.y}px`;
  }

  /**
   * Hide "Ask about this" button.
   */
  hideAskButton() {
    this.askButton.style.display = 'none';
  }

  /**
   * Ask question about selected text.
   */
  askAboutSelected() {
    if (!this.selectedText) return;

    const validation = TextSelectionUtils.validateSelectedText(this.selectedText);
    if (!validation.valid) {
      this.setStatus(`Error: ${validation.error}`, 'error');
      return;
    }

    // Auto-populate query input
    this.queryInput.value = 'What is the significance of this passage?';
    this.queryInput.focus();

    // Switch to selected mode
    this.currentMode = 'selected';
    document.querySelector('input[value="selected"]').checked = true;
    this.updateModeUI();

    this.hideAskButton();
    this.setStatus('Ready to ask about selected text');
  }

  /**
   * Send query to chatbot.
   */
  async sendQuery() {
    const query = this.queryInput.value.trim();
    if (!query) {
      this.setStatus('Please enter a question', 'warning');
      return;
    }

    if (this.isLoading) return;

    if (this.currentMode === 'selected' && !this.selectedText) {
      this.setStatus('Please select text to query', 'warning');
      return;
    }

    this.isLoading = true;
    this.setStatus('Thinking...');

    // Add user message
    const userMsg = MessageFormatter.formatMessage(query, 'user');
    this.messages.push(userMsg);
    this.renderMessages();
    this.queryInput.value = '';
    this.queryInput.focus();

    try {
      const request = {
        query,
        book_id: this.bookId,
        mode: this.currentMode,
      };

      if (this.currentMode === 'selected') {
        request.selected_text = this.selectedText;
      }

      let finalResponse;
      if (this.enableStreaming) {
        finalResponse = await this.chatWithStreaming(request);
      } else {
        const response = await this.apiClient.chat(request);
        const assistantMsg = MessageFormatter.formatMessage(
          response.response_text,
          'assistant'
        );
        assistantMsg.confidence = response.confidence;
        assistantMsg.latency = response.latency_ms;
        this.messages.push(assistantMsg);
        finalResponse = response;
      }

      this.saveMessages();
      this.renderMessages();

      const confidence = MessageFormatter.formatConfidence(finalResponse.confidence);
      const latency = MessageFormatter.formatLatency(finalResponse.latency_ms);
      this.setStatus(`Confidence: ${confidence} | Latency: ${latency}`);

    } catch (error) {
      this.setStatus(`Error: ${error.message}`, 'error');
      // The streaming function handles its own error message display
      if (!this.enableStreaming) {
        this.messages.pop(); // Remove the user message if the request failed outright
      }
      this.renderMessages();
    } finally {
      this.isLoading = false;
    }
  }

  /**
   * Stream chat response from server.
   *
   * @param {object} request - Chat request
   * @returns {Promise<object>} Final response object
   */
  async chatWithStreaming(request) {
    // 1. Add a placeholder for the assistant's message
    const assistantMsg = MessageFormatter.formatMessage('...', 'assistant');
    this.messages.push(assistantMsg);
    this.renderMessages();

    let fullContent = '';
    let metadata = {};

    return new Promise((resolve, reject) => {
      const onChunk = (chunk) => {
        try {
          // Handle different chunk types
          if (chunk.type === 'error') {
            assistantMsg.text = `Error: ${chunk.message}`;
            assistantMsg.isError = true;
            this.renderMessages();
            reject(new Error(chunk.message));
            return;
          }

          if (chunk.type === 'text') {
            fullContent += chunk.content;
            assistantMsg.text = MessageFormatter.formatResponseWithCitations({ response: fullContent, ...metadata });
          } else {
            // Collect metadata from non-text chunks
            Object.assign(metadata, chunk);
            assistantMsg.confidence = metadata.confidence;
            assistantMsg.latency = metadata.latency_ms; // Corrected latency property name
          }

          // Re-render the message with updated content/metadata
          this.renderMessages();
          this.messagesContainer.scrollTop = this.messagesContainer.scrollHeight;

        } catch (e) {
          console.error('Error processing chunk:', e);
          assistantMsg.text = `Error: Malformed data received from server.`;
          assistantMsg.isError = true;
          this.renderMessages();
          reject(new Error('Malformed data from server.'));
        }
      };

      this.apiClient
        .chatStream(request, onChunk)
        .then(() => {
          // Finalize the message content and metadata
          assistantMsg.text = MessageFormatter.formatResponseWithCitations({ response: fullContent, ...metadata });
          Object.assign(assistantMsg, metadata);
          this.saveMessages();
          this.renderMessages();
          resolve({ response: fullContent, ...metadata });
        })
        .catch((err) => {
          assistantMsg.text = `Error: ${err.message}`;
          assistantMsg.isError = true;
          this.renderMessages();
          reject(err);
        });
    });
  }

  /**
   * Update streaming message in real-time.
   *
   * @param {string} text - Current response text
   */
  updateStreamingMessage(text) {
    // This function is no longer the primary method for updating the display,
    // as the new `chatWithStreaming` handles it directly.
    // It can be kept for now or removed if no longer used elsewhere.
    const lastMsg = this.messages[this.messages.length - 1];
    if (lastMsg && lastMsg.role === 'assistant') {
      lastMsg.text = text;
      this.renderMessages();
      this.messagesContainer.scrollTop = this.messagesContainer.scrollHeight;
    }
  }

  /**
   * Render all messages.
   */
  renderMessages() {
    this.messagesContainer.innerHTML = this.messages
      .map((msg) => this.renderMessage(msg))
      .join('');

    // Auto-scroll to bottom
    this.messagesContainer.scrollTop = this.messagesContainer.scrollHeight;
  }

  /**
   * Render single message.
   *
   * @param {object} msg - Message object
   * @returns {string} HTML string
   */
  renderMessage(msg) {
    const roleClass = `rag-widget__message--${msg.role}`;
    const errorClass = msg.isError ? 'rag-widget__message--error' : '';
    const confidenceHTML =
      msg.confidence !== undefined
        ? `<small class="rag-widget__confidence">
             Confidence: ${MessageFormatter.formatConfidence(msg.confidence)}
           </small>`
        : '';

    return `
      <div class="rag-widget__message ${roleClass} ${errorClass}">
        <div class="rag-widget__message-content">
          ${this.escapeHTML(msg.text)}
        </div>
        ${confidenceHTML}
      </div>
    `;
  }

  /**
   * Escape HTML in text.
   *
   * @param {string} text - Text to escape
   * @returns {string} Escaped HTML
   */
  escapeHTML(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
  }

  /**
   * Update mode UI.
   */
  updateModeUI() {
    if (this.currentMode === 'selected' && !this.selectedText) {
      this.setStatus('Select text in the book to ask about it', 'info');
    } else {
      this.setStatus('');
    }
  }

  /**
   * Set status message.
   *
   * @param {string} message - Status message
   * @param {string} type - 'info', 'warning', 'error'
   */
  setStatus(message, type = 'info') {
    this.statusDiv.textContent = message;
    this.statusDiv.className = `rag-widget__status rag-widget__status--${type}`;
  }

  /**
   * Clear message history.
   */
  clearHistory() {
    if (confirm('Clear all messages?')) {
      this.messages = [];
      this.renderMessages();
      if (this.enableHistory) {
        StorageManager.clearMessages(this.bookId);
      }
      this.setStatus('History cleared');
    }
  }

  /**
   * Save messages to localStorage.
   */
  saveMessages() {
    if (this.enableHistory) {
      StorageManager.saveMessages(this.bookId, this.messages);
    }
  }
}

// Auto-initialize if data-widget attribute found
document.addEventListener('DOMContentLoaded', () => {
  const widgets = document.querySelectorAll('[data-rag-widget]');
  widgets.forEach((el) => {
    const bookId = el.getAttribute('data-book-id');
    if (bookId) {
      new RAGChatWidget({
        bookId,
        containerId: el.id || undefined,
      });
    }
  });
});

// Export for use as module
if (typeof module !== 'undefined' && module.exports) {
  module.exports = RAGChatWidget;
}
  window.RAGChatWidget = RAGChatWidget;
} // Close the if (typeof window.RAGChatWidget === 'undefined') block
