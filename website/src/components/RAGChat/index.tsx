/**
 * Enhanced RAGChat Component with Selected Text Support
 *
 * A chat interface that supports both full-book and selected-text modes
 */
import React, { useState, useEffect, useRef } from 'react';
import { useRAGChat } from '../../hooks/useRAGChat';
import styles from './RAGChat.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: number;
}

export const RAGChat: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const {
    messages,
    isLoading,
    error,
    selectedText,
    mode,
    sendQuery,
    setSelectedText
  } = useRAGChat();

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Set up text selection handler
  useEffect(() => {
    if (isOpen) {
      const handleSelection = () => {
        const selection = window.getSelection();
        if (selection && selection.toString().trim().length > 0) {
          const selectedText = selection.toString().trim();
          // Only set if selection is substantial (min 10 chars)
          if (selectedText.length >= 10) {
            setSelectedText(selectedText);
          }
        }
      };

      document.addEventListener('mouseup', handleSelection);
      document.addEventListener('touchend', handleSelection);

      return () => {
        document.removeEventListener('mouseup', handleSelection);
        document.removeEventListener('touchend', handleSelection);
      };
    }
  }, [isOpen, setSelectedText]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = (query: string) => {
    sendQuery(query);
  };

  return (
    <div className={styles.chatContainer}>
      {!isOpen ? (
        <button
          className={styles.floatingButton}
          onClick={() => setIsOpen(true)}
          aria-label="Open chat"
        >
          💬
        </button>
      ) : (
        <div className={styles.chatModal}>
          <div className={styles.chatHeader}>
            <h3 className={styles.chatTitle}>Physical AI Chat</h3>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          {error && (
            <div className={styles.errorBanner}>
              Error: {error}
            </div>
          )}

          {/* Selected text indicator */}
          {selectedText && (
            <div className={styles.selectedTextIndicator}>
              <div className={styles.selectedTextHeader}>
                <span className={styles.selectedTextLabel}>Selected Text Mode</span>
                <button
                  className={styles.clearSelectedText}
                  onClick={() => setSelectedText(null)}
                  title="Clear selection"
                >
                  ✕
                </button>
              </div>
              <div className={styles.selectedTextPreview}>
                "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
              </div>
            </div>
          )}

          <div className={styles.chatMessages}>
            {messages.length === 0 ? (
              <div style={{ textAlign: 'center', color: '#6b7280', marginTop: '20px' }}>
                Ask me anything about the Physical AI textbook!
                {selectedText && <div style={{ marginTop: '8px', fontSize: '14px', color: '#4f46e5' }}>
                  Currently in "selected text" mode
                </div>}
              </div>
            ) : (
              messages.map((msg) => (
                <div
                  key={msg.id}
                  className={`${styles.message} ${msg.role === 'user' ? styles.userMessage : styles.assistantMessage}`}
                >
                  {msg.content}
                </div>
              ))
            )}

            {isLoading && (
              <div className={styles.loadingIndicator}>
                Thinking...
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <ChatInput
            onSend={handleSendMessage}
            disabled={isLoading}
            selectedText={selectedText}
            mode={mode}
          />
        </div>
      )}
    </div>
  );
};

// Input component
interface ChatInputProps {
  onSend: (message: string) => void;
  disabled: boolean;
  selectedText: string | null;
  mode: 'full' | 'selected';
}

const ChatInput: React.FC<ChatInputProps> = ({ onSend, disabled, selectedText, mode }) => {
  const [inputValue, setInputValue] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (inputValue.trim() && !disabled) {
      onSend(inputValue);
      setInputValue('');
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e as any);
    }
  };

  // Default placeholder based on mode
  const placeholder = selectedText
    ? "Ask about the selected text..."
    : "Type your question...";

  return (
    <div className={styles.chatInput}>
      <form onSubmit={handleSubmit} className={styles.inputForm}>
        <textarea
          className={styles.inputField}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={placeholder}
          rows={2}
          disabled={disabled}
        />
        <button
          type="submit"
          className={styles.sendButton}
          disabled={disabled || !inputValue.trim()}
        >
          Send
        </button>
      </form>

      {selectedText && (
        <div className={styles.modeIndicator}>
          <span className={styles.modeLabel}>Mode: Selected Text</span>
        </div>
      )}
    </div>
  );
};

export default RAGChat;