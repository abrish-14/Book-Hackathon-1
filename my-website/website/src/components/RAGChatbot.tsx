import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './RAGChatbot.module.css';

interface RAGChatbotProps {
  className?: string;
}

const RAGChatbot: React.FC<RAGChatbotProps> = ({ className }) => {
  const [messages, setMessages] = useState<{id: number, text: string, sender: 'user' | 'bot'}[]>([]);
  const [inputValue, setInputValue] = useState('');

  const handleSendMessage = () => {
    if (inputValue.trim() === '') return;

    // Add user message
    const newUserMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user' as const
    };

    setMessages(prev => [...prev, newUserMessage]);
    setInputValue('');

    // Simulate bot response (placeholder for actual RAG logic)
    setTimeout(() => {
      const botResponse = {
        id: Date.now() + 1,
        text: `This is a placeholder response. Actual RAG integration will provide context-aware answers based on the documentation.`,
        sender: 'bot' as const
      };
      setMessages(prev => [...prev, botResponse]);
    }, 1000);
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className={clsx('rag-chatbot-container', className, styles.container)}>
      <div className={styles.chatHeader}>
        <h3>AI Documentation Assistant</h3>
        <p>Powered by RAG (Retrieval-Augmented Generation)</p>
      </div>

      <div className={styles.chatMessages}>
        {messages.length === 0 ? (
          <div className={styles.welcomeMessage}>
            <p>Hello! I'm your AI documentation assistant.</p>
            <p>Ask me questions about the AI/Spec-Driven Book content, and I'll provide answers based on the documentation.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={clsx(
                styles.message,
                message.sender === 'user' ? styles.userMessage : styles.botMessage
              )}
            >
              <div className={styles.messageContent}>
                {message.text}
              </div>
              <div className={clsx(styles.sender, styles[`${message.sender}Label`])}>
                {message.sender === 'user' ? 'You' : 'AI Assistant'}
              </div>
            </div>
          ))
        )}
      </div>

      <div className={styles.chatInput}>
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about the documentation..."
          className={styles.inputField}
          rows={3}
        />
        <button
          onClick={handleSendMessage}
          disabled={!inputValue.trim()}
          className={clsx(styles.sendButton, !inputValue.trim() && styles.disabled)}
        >
          Send
        </button>
      </div>

      <div className={styles.chatFooter}>
        <p>Note: This is a placeholder implementation. Full RAG functionality will be integrated in future development.</p>
      </div>
    </div>
  );
};

export default RAGChatbot;