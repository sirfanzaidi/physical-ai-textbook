import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';
import { getApiUrl } from '@/lib/apiConfig';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  timestamp: Date;
}

interface Citation {
  chapter: number;
  title: string;
  content: string;
  relevance_score: number;
}

export default function ChatBot(): JSX.Element {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Listen for text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 10) {
        setSelectedText(text);
        setIsOpen(true);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const sendMessage = async (question?: string) => {
    const messageText = question || input;
    if (!messageText.trim() || isLoading) return;

    const userMessage: Message = {
      role: 'user',
      content: messageText,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // Call backend RAG API
      const apiUrl = getApiUrl();
      const response = await fetch(`${apiUrl}/api/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: messageText,
          context: selectedText || undefined,
          top_k: 5,
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, assistantMessage]);
      setSelectedText(''); // Clear selected text after use
    } catch (error) {
      console.error('Chat error:', error);
      const apiUrl = getApiUrl();
      const errorMessage: Message = {
        role: 'assistant',
        content: `Sorry, I encountered an error. Please make sure the backend server is running at ${apiUrl}`,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const clearChat = () => {
    setMessages([]);
    setSelectedText('');
  };

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chatbot"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <h3>📚 Physical AI Assistant</h3>
            <button onClick={clearChat} className={styles.clearButton}>
              Clear
            </button>
          </div>

          {/* Selected Text Context */}
          {selectedText && (
            <div className={styles.contextBanner}>
              <strong>Context:</strong> "{selectedText.substring(0, 100)}..."
              <button onClick={() => setSelectedText('')}>✕</button>
            </div>
          )}

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 Hi! I'm your Physical AI textbook assistant.</p>
                <p>Ask me anything about:</p>
                <ul>
                  <li>Embodied AI & Physical Intelligence</li>
                  <li>Humanoid Robotics & Locomotion</li>
                  <li>ROS2 Architecture & Tools</li>
                  <li>Digital Twins & Simulation</li>
                  <li>Vision-Language-Action Models</li>
                </ul>
                <p><em>Tip: Select text from the textbook, and I'll answer questions about it!</em></p>
              </div>
            )}

            {messages.map((message, index) => (
              <div
                key={index}
                className={
                  message.role === 'user'
                    ? styles.userMessage
                    : styles.assistantMessage
                }
              >
                <div className={styles.messageContent}>
                  <strong>{message.role === 'user' ? 'You' : 'Assistant'}:</strong>
                  <p>{message.content}</p>

                  {/* Citations */}
                  {message.citations && message.citations.length > 0 && (
                    <div className={styles.citations}>
                      <strong>Sources:</strong>
                      {message.citations.map((citation, i) => (
                        <div key={i} className={styles.citation}>
                          <a href={`/docs/0${citation.chapter}-${citation.title.toLowerCase().replace(/ /g, '-')}`}>
                            Chapter {citation.chapter}: {citation.title}
                          </a>
                          <span className={styles.relevanceScore}>
                            {(citation.relevance_score * 100).toFixed(0)}% relevant
                          </span>
                        </div>
                      ))}
                    </div>
                  )}

                  <span className={styles.timestamp}>
                    {message.timestamp.toLocaleTimeString()}
                  </span>
                </div>
              </div>
            ))}

            {isLoading && (
              <div className={styles.assistantMessage}>
                <div className={styles.messageContent}>
                  <strong>Assistant:</strong>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.inputContainer}>
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about Physical AI..."
              rows={2}
              disabled={isLoading}
              className={styles.input}
            />
            <button
              onClick={() => sendMessage()}
              disabled={isLoading || !input.trim()}
              className={styles.sendButton}
            >
              {isLoading ? '⏳' : '➤'}
            </button>
          </div>
        </div>
      )}
    </>
  );
}
