/**
 * InputBox Component
 *
 * Text input for queries with character counter and send button.
 */

import React, { useState } from 'react';
import { ragConfig } from '../../config/ragConfig';
import styles from './RAGChat.module.css';

interface InputBoxProps {
  onSubmit: (query: string) => void;
  isLoading: boolean;
  selectedText?: string | null;
}

export const InputBox: React.FC<InputBoxProps> = ({ onSubmit, isLoading, selectedText }) => {
  const [query, setQuery] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (query.trim() && !isLoading) {
      onSubmit(query);
      setQuery('');
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    // Submit on Ctrl+Enter or Cmd+Enter
    if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') {
      handleSubmit(e as any);
    }
  };

  const charCount = query.length;
  const isOverLimit = charCount > ragConfig.maxQueryLength;
  const isDisabled = isLoading || isOverLimit || !query.trim();

  return (
    <div className={styles.inputBox}>
      {selectedText && (
        <div className={styles.selectedTextIndicator}>
          <span className={styles.selectedTextLabel}>Selected text mode</span>
          <span className={styles.selectedTextPreview}>{selectedText.substring(0, 50)}...</span>
        </div>
      )}

      <form onSubmit={handleSubmit} className={styles.inputForm}>
        <textarea
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask me something... (Ctrl+Enter to send)"
          className={styles.textarea}
          disabled={isLoading}
          rows={3}
        />

        <div className={styles.inputFooter}>
          <div className={`${styles.charCounter} ${isOverLimit ? styles.counterError : ''}`}>
            {charCount} / {ragConfig.maxQueryLength}
          </div>

          <button
            type="submit"
            disabled={isDisabled}
            className={styles.submitButton}
            title={isLoading ? 'Waiting for response...' : 'Send query (Ctrl+Enter)'}
          >
            {isLoading ? (
              <>
                <span className={styles.loadingSpinner}>⏳</span>
                Sending...
              </>
            ) : (
              <>
                <span>→</span>
                Send
              </>
            )}
          </button>
        </div>
      </form>
    </div>
  );
};
