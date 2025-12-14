/**
 * RAG Chat Component for Docusaurus
 *
 * Integrates the Vanilla JS RAG chat widget into the Docusaurus site.
 * Loads frontend files (chat-widget.js, utils.js, chat-widget.css) and
 * initializes the widget with appropriate configuration.
 */

import React, { useEffect, useRef } from 'react';
import styles from './RAGChat.module.css';

interface RAGChatProps {
  /**
   * Book ID to query (e.g., 'physical-ai-textbook')
   * Determines which indexed book the chatbot will search
   */
  bookId: string;

  /**
   * API base URL (e.g., 'http://localhost:8000' or 'https://api.example.com')
   * Defaults to the Docusaurus site's base URL
   */
  apiBaseURL?: string;

  /**
   * Enable streaming responses (default: true)
   * Shows response text appearing character-by-character
   */
  enableStreaming?: boolean;

  /**
   * Enable message history persistence (default: true)
   * Saves conversation to localStorage
   */
  enableHistory?: boolean;

  /**
   * Container height (default: '600px')
   * Can be customized for different page layouts
   */
  height?: string;
}

// Global flag to prevent duplicate script loading
let scriptsLoadedGlobally = false;

/**
 * RAG Chat React Component
 *
 * Wraps the Vanilla JS RAG chatbot widget for use in Docusaurus.
 * Handles dynamic script loading, CSS loading, and widget initialization.
 *
 * Usage:
 * ```tsx
 * <RAGChat
 *   bookId="physical-ai-textbook"
 *   apiBaseURL="https://api.example.com"
 *   height="600px"
 * />
 * ```
 */
const RAGChat: React.FC<RAGChatProps> = ({
  bookId,
  apiBaseURL,
  enableStreaming = true,
  enableHistory = true,
  height = '600px',
}) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const initializedRef = useRef(false);
  const containerId = `rag-widget-container`;

  useEffect(() => {
    // Skip if already initialized
    if (initializedRef.current) return;

    const initializeChat = () => {
      try {
        // Check if scripts are already loaded
        const hasUtils = typeof (window as any).APIClient !== 'undefined';
        const hasWidget = typeof (window as any).RAGChatWidget !== 'undefined';

        if (!hasUtils || !hasWidget) {
          // Scripts not loaded yet, need to load them
          if (!scriptsLoadedGlobally) {
            // 1. Load CSS stylesheet (only once)
            if (!document.getElementById('rag-chat-styles')) {
              const cssLink = document.createElement('link');
              cssLink.id = 'rag-chat-styles';
              cssLink.rel = 'stylesheet';
              cssLink.href = '/chat-widget.css';
              document.head.appendChild(cssLink);
            }

            // 2. Load utility functions (utils.js)
            if (!document.getElementById('rag-utils-script')) {
              const utilsScript = document.createElement('script');
              utilsScript.id = 'rag-utils-script';
              utilsScript.src = '/utils.js';
              utilsScript.async = true;
              document.head.appendChild(utilsScript);
            }

            // 3. Load chat widget (chat-widget.js)
            if (!document.getElementById('rag-widget-script')) {
              const chatScript = document.createElement('script');
              chatScript.id = 'rag-widget-script';
              chatScript.src = '/chat-widget.js';
              chatScript.async = false;
              document.head.appendChild(chatScript);

              // Wait for scripts to fully load
              chatScript.onload = () => {
                scriptsLoadedGlobally = true;
                // Give time for scripts to execute
                setTimeout(initializeWidgetInstance, 100);
              };

              chatScript.onerror = () => {
                console.error('Failed to load RAG chat widget scripts');
              };
            }
          } else {
            // Scripts already loaded globally, just initialize the widget
            setTimeout(initializeWidgetInstance, 100);
          }
        } else {
          // Scripts already available, initialize immediately
          initializeWidgetInstance();
        }
      } catch (error) {
        console.error('Error initializing RAG chat widget:', error);
      }
    };

    const initializeWidgetInstance = () => {
      // Check if already initialized
      if (initializedRef.current) return;

      // Check if RAGChatWidget class is available
      if (typeof (window as any).RAGChatWidget === 'undefined') {
        console.error('RAGChatWidget class not available');
        return;
      }

      // Check if container exists in DOM
      const container = document.getElementById(containerId);
      if (!container) {
        console.error(`Container ${containerId} not found in DOM`);
        return;
      }

      try {
        new (window as any).RAGChatWidget({
          bookId,
          containerId,
          apiBaseURL: apiBaseURL || window.location.origin,
          enableStreaming,
          enableHistory,
        });
        initializedRef.current = true;
        console.log('RAGChatWidget initialized successfully');
      } catch (error) {
        console.error('Error instantiating RAGChatWidget:', error);
      }
    };

    initializeChat();

    // Cleanup
    return () => {
      // Optional: cleanup if needed
    };
  }, [bookId, apiBaseURL, enableStreaming, enableHistory]);

  return (
    <div className={styles.ragChatWrapper}>
      <div
        id={containerId}
        ref={containerRef}
        className={styles.ragChatContainer}
        style={{ height }}
      />
    </div>
  );
};

export default RAGChat;
