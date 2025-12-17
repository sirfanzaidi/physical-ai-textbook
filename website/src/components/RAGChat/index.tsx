/**
 * RAGChat Container Component
 *
 * Main component combining chat window, input, and floating button.
 * Manages chat state and handles all user interactions.
 */

import React, { useState, useEffect } from 'react';
import { useRAGChat } from '../../hooks/useRAGChat';
import { ChatWindow } from './ChatWindow';
import { InputBox } from './InputBox';
import { FloatingButton } from './FloatingButton';
import styles from './RAGChat.module.css';

interface RAGChatProps {
  bookId?: string;
  autoOpen?: boolean;
}

export const RAGChat: React.FC<RAGChatProps> = ({ bookId = 'physical-ai', autoOpen = false }) => {
  const [isOpen, setIsOpen] = useState(autoOpen);
  const {
    messages,
    isLoading,
    error,
    selectedText,
    sendQuery,
    setSelectedText,
    clearHistory,
  } = useRAGChat();

  // Close on Escape key
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        setIsOpen(false);
      }
    };

    document.addEventListener('keydown', handleEscape);
    return () => document.removeEventListener('keydown', handleEscape);
  }, [isOpen]);

  // Handle text selection for US2 (Ask about selected text)
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().length > 0) {
        const selectedText = selection.toString();
        // Only set if selection is substantial (min 10 chars)
        if (selectedText.length >= 10) {
          setSelectedText(selectedText);
        }
      }
    };

    // Detect selection when modal is open
    if (isOpen) {
      document.addEventListener('mouseup', handleSelection);
      document.addEventListener('touchend', handleSelection);
      return () => {
        document.removeEventListener('mouseup', handleSelection);
        document.removeEventListener('touchend', handleSelection);
      };
    }
  }, [isOpen, setSelectedText]);

  return (
    <>
      {/* Floating button */}
      <FloatingButton
        isOpen={isOpen}
        onClick={() => setIsOpen(!isOpen)}
        unreadCount={0}
      />

      {/* Modal backdrop */}
      {isOpen && (
        <div
          className={styles.backdrop}
          onClick={() => setIsOpen(false)}
          aria-hidden="true"
        />
      )}

      {/* Chat modal */}
      {isOpen && (
        <div className={styles.chatModal} role="dialog" aria-label="Chat with AI">
          {/* Header */}
          <div className={styles.header}>
            <div className={styles.headerContent}>
              <h3 className={styles.headerTitle}>Physical AI Chat</h3>
              <p className={styles.headerSubtitle}>Ask about the textbook</p>
            </div>
            <div className={styles.headerActions}>
              {messages.length > 0 && (
                <button
                  className={styles.clearButton}
                  onClick={clearHistory}
                  title="Clear chat history"
                  aria-label="Clear conversation"
                >
                  🔄
                </button>
              )}
              <button
                className={styles.closeButton}
                onClick={() => setIsOpen(false)}
                title="Close chat"
                aria-label="Close chat"
              >
                ✕
              </button>
            </div>
          </div>

          {/* Error message */}
          {error && (
            <div className={styles.errorBanner}>
              <span>⚠️ {error}</span>
              <button
                className={styles.closeBanner}
                onClick={() => {
                  /* Error is cleared on next message */
                }}
              >
                ✕
              </button>
            </div>
          )}

          {/* Chat window */}
          <ChatWindow messages={messages} isLoading={isLoading} />

          {/* Input box */}
          <InputBox
            onSubmit={sendQuery}
            isLoading={isLoading}
            selectedText={selectedText}
          />

          {/* Footer */}
          <div className={styles.footer}>
            <p className={styles.footerText}>
              Powered by <strong>Physical AI</strong> • Answers from the textbook
            </p>
          </div>
        </div>
      )}
    </>
  );
};

export default RAGChat;
