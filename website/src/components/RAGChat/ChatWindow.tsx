/**
 * ChatWindow Component
 *
 * Displays conversation messages with citations and metadata.
 * Auto-scrolls to latest message and shows loading state.
 */

import React, { useEffect, useRef } from 'react';
import { Message } from '../../hooks/useRAGChat';
import { Citation } from './Citation';
import styles from './RAGChat.module.css';

interface ChatWindowProps {
  messages: Message[];
  isLoading: boolean;
}

export const ChatWindow: React.FC<ChatWindowProps> = ({ messages, isLoading }) => {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <div className={styles.chatWindow}>
      {messages.length === 0 ? (
        <div className={styles.emptyState}>
          <div className={styles.emptyIcon}>💬</div>
          <p>Ask me anything about the Physical AI textbook!</p>
          <p className={styles.emptyHint}>Type your question and press Enter</p>
        </div>
      ) : (
        messages.map((message) => (
          <div key={message.id} className={`${styles.message} ${styles[message.role]}`}>
            <div className={styles.messageContent}>
              <div className={styles.messageText}>{message.content}</div>

              {/* Show citations for assistant messages */}
              {message.citations && message.citations.length > 0 && (
                <div className={styles.citations}>
                  <div className={styles.citationsLabel}>Sources:</div>
                  <div className={styles.citationsList}>
                    {message.citations.map((citation) => (
                      <Citation key={citation.chunk_id} citation={citation} />
                    ))}
                  </div>
                </div>
              )}

              {/* Show metadata for assistant messages */}
              {message.metadata && (
                <div className={styles.metadata}>
                  {message.metadata.latency_ms && (
                    <span>{(message.metadata.latency_ms / 1000).toFixed(2)}s</span>
                  )}
                  {message.metadata.model && (
                    <span className={styles.model}>{message.metadata.model.split('/').pop()}</span>
                  )}
                </div>
              )}
            </div>
          </div>
        ))
      )}

      {/* Loading indicator */}
      {isLoading && (
        <div className={`${styles.message} ${styles.assistant}`}>
          <div className={styles.messageContent}>
            <div className={styles.loadingDots}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        </div>
      )}

      <div ref={messagesEndRef} />
    </div>
  );
};
