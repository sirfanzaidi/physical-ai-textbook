/**
 * RAG Chatbot Page
 *
 * Dedicated page for the RAG chatbot widget with full-screen layout.
 * Allows readers to ask questions about the Physical AI textbook content.
 */

import React from 'react';
import Layout from '@theme/Layout';
import RAGChat from '@site/src/components/RAGChat';
import styles from './chat.module.css';

const ChatPage: React.FC = () => {
  return (
    <Layout
      title="Ask the Textbook"
      description="Interactive RAG chatbot for the Physical AI & Humanoid Robotics textbook"
    >
      <main className={styles.chatPageContainer}>
        <div className={styles.chatHeader}>
          <h1>Ask the Textbook</h1>
          <p>
            Powered by Retrieval-Augmented Generation (RAG), this chatbot can answer questions
            about the Physical AI & Humanoid Robotics textbook content with cited sources.
          </p>
        </div>

        <div className={styles.chatContent}>
          <RAGChat />
        </div>

        <div className={styles.chatInfo}>
          <div className={styles.infoSection}>
            <h2>How to Use</h2>
            <ul>
              <li>
                <strong>Full Book Mode</strong>: Ask questions about any part of the textbook. The
                AI will search the entire indexed content and return answers with sources.
              </li>
              <li>
                <strong>Select Text Mode</strong>: Highlight any text in the textbook and click
                "Ask about this" to ask questions about only that specific passage.
              </li>
              <li>
                <strong>Message History</strong>: Your conversation history is saved in your
                browser. Clear it anytime using the "Clear" button.
              </li>
            </ul>
          </div>

          <div className={styles.infoSection}>
            <h2>Features</h2>
            <ul>
              <li>✓ AI-powered semantic search across the textbook</li>
              <li>✓ Automatic source citations with chapter and page numbers</li>
              <li>✓ Streaming responses for real-time feedback</li>
              <li>✓ Zero-leakage select-text mode for precise queries</li>
              <li>✓ Message history persistence in your browser</li>
              <li>✓ Responsive design for all devices</li>
              <li>✓ Dark mode support</li>
            </ul>
          </div>

          <div className={styles.infoSection}>
            <h2>Example Queries</h2>
            <div className={styles.examples}>
              <div className={styles.example}>
                <strong>Full Book:</strong>
                <p>"What are the main components of a humanoid robot?"</p>
              </div>
              <div className={styles.example}>
                <strong>Full Book:</strong>
                <p>"Explain the ROS 2 node architecture"</p>
              </div>
              <div className={styles.example}>
                <strong>Select Text:</strong>
                <p>Highlight a passage and ask "Summarize this section"</p>
              </div>
            </div>
          </div>

          <div className={styles.infoSection}>
            <h2>Technical Details</h2>
            <p>
              This chatbot uses <strong>Retrieval-Augmented Generation (RAG)</strong> to provide
              accurate, grounded answers directly from the textbook content. It combines:
            </p>
            <ul>
              <li>
                <strong>Semantic Search</strong>: Qdrant vector database with OpenRouter embeddings (Qwen)
              </li>
              <li>
                <strong>Relevance Ranking</strong>: Semantic similarity scoring for precision
              </li>
              <li>
                <strong>Grounded Generation</strong>: OpenRouter LLM with document context
              </li>
              <li>
                <strong>Zero Hallucination</strong>: Constrained to textbook content only
              </li>
            </ul>
          </div>

          <div className={styles.infoSection}>
            <h2>Privacy & Storage</h2>
            <p>
              Your message history is stored only in your browser's localStorage. No personal data
              is sent to external services beyond the AI API calls needed to generate responses. The
              chatbot does not track your questions or build user profiles.
            </p>
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default ChatPage;
