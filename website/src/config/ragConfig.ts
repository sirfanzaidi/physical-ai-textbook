/**
 * RAG Chatbot Configuration
 *
 * Handles environment-specific API URLs and chat configuration.
 * Automatically detects development vs. production environments.
 */

export interface RAGChatConfig {
  /** Book ID to index and query */
  bookId: string;

  /** API base URL for backend */
  apiBaseURL: string;

  /** Enable streaming responses */
  enableStreaming: boolean;

  /** Enable message history persistence */
  enableHistory: boolean;

  /** Default container height */
  defaultHeight: string;
}

/**
 * Get RAG chat configuration based on environment
 *
 * Development: Uses localhost:8000
 * Production: Uses relative /api path (configured via vercel.json)
 * Staging: Uses staging API URL
 */
export const getRagConfig = (): RAGChatConfig => {
  // Determine environment
  const isDevelopment =
    typeof window !== 'undefined' &&
    (window.location.hostname === 'localhost' ||
      window.location.hostname === '127.0.0.1' ||
      window.location.hostname.startsWith('192.168'));

  const isProduction =
    typeof window !== 'undefined' && window.location.hostname.includes('vercel.app');

  // Determine API base URL
  let apiBaseURL = '/api'; // Default to relative path (vercel.json rewrites)

  if (isDevelopment) {
    apiBaseURL = 'http://localhost:8000';
  } else if (isProduction) {
    // Use Vercel rewrites configured in vercel.json
    apiBaseURL = '/api';
  }

  return {
    bookId: 'physical-ai-textbook',
    apiBaseURL,
    enableStreaming: true,
    enableHistory: true,
    defaultHeight: '700px',
  };
};

/**
 * Default configuration
 */
export const defaultRagConfig: RAGChatConfig = {
  bookId: 'physical-ai-textbook',
  apiBaseURL: '/api',
  enableStreaming: true,
  enableHistory: true,
  defaultHeight: '700px',
};
