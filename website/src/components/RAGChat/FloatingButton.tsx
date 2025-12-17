/**
 * FloatingButton Component
 *
 * Floating action button to toggle chat modal.
 * Positioned bottom-right, responsive for mobile.
 */

import React from 'react';
import styles from './RAGChat.module.css';

interface FloatingButtonProps {
  isOpen: boolean;
  onClick: () => void;
  unreadCount?: number;
}

export const FloatingButton: React.FC<FloatingButtonProps> = ({
  isOpen,
  onClick,
  unreadCount = 0,
}) => {
  return (
    <button
      className={`${styles.floatingButton} ${isOpen ? styles.open : ''}`}
      onClick={onClick}
      title={isOpen ? 'Close chat' : 'Open chat'}
      aria-label="Toggle chat widget"
      aria-pressed={isOpen}
    >
      {/* Chat bubble icon */}
      <div className={styles.buttonIcon}>
        {isOpen ? (
          <span>✕</span>
        ) : (
          <>
            <span className={styles.chatBubble}>💬</span>
            {unreadCount > 0 && (
              <div className={styles.badge}>{unreadCount > 99 ? '99+' : unreadCount}</div>
            )}
          </>
        )}
      </div>

      {/* Tooltip */}
      <div className={styles.tooltip}>{isOpen ? 'Close' : 'Chat with AI'}</div>
    </button>
  );
};
