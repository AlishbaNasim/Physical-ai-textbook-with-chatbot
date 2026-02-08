import React, { useState, useRef, useEffect } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import './ChatbotWidget.css'; // Import the CSS file

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);

  // Initialize session ID only on client side
  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
      const storedSessionId = localStorage.getItem('chatbot-session-id');
      if (storedSessionId) {
        setSessionId(storedSessionId);
      } else {
        const newSessionId = `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
        localStorage.setItem('chatbot-session-id', newSessionId);
        setSessionId(newSessionId);
      }
    }
  }, []);

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current.focus(), 100);
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Get current page context from window location
      const context = {
        currentPage: ExecutionEnvironment.canUseDOM ? window.location.pathname : '/',
        userPreferences: {
          experienceLevel: 'intermediate',
          interests: ['robotics', 'ai', 'physical-ai']
        }
      };

      // Send request to backend API
      const response = await fetch(`${process.env.REACT_APP_CHATBOT_API_URL || '/api'}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${process.env.REACT_APP_API_KEY || 'dummy-key'}`, // In production, use proper auth
        },
        body: JSON.stringify({
          query: inputValue,
          sessionId: sessionId,
          context: context
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Add bot response to chat
      const botMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'bot',
        sources: data.sources,
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Chat error:', error);

      // Add error message to chat
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'bot',
        isError: true,
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  return (
    <div className="chatbot-container">
      {/* Floating button */}
      {!isOpen && (
        <button
          className="chatbot-button"
          onClick={toggleChat}
          aria-label="Open chatbot"
          title="Ask questions about the textbook content"
        >
          <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        </button>
      )}

      {/* Chat window */}
      {isOpen && (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <div className="chatbot-title">Physical AI Assistant</div>
            <div className="chatbot-subtitle">Ask questions about the textbook content</div>
            <button
              className="chatbot-close"
              onClick={toggleChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="chatbot-welcome">
                <p>Hello! I'm your Physical AI & Humanoid Robotics assistant.</p>
                <p>You can ask me questions about the textbook content, and I'll provide answers based on the material.</p>
                <p>Try asking about:</p>
                <ul>
                  <li>ROS 2 concepts</li>
                  <li>Simulation techniques</li>
                  <li>Vision-Language-Action systems</li>
                  <li>Humanoid robot control</li>
                </ul>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`chatbot-message ${message.sender}-message`}
                >
                  <div className="message-content">
                    <p>{message.text}</p>

                    {message.sources && message.sources.length > 0 && (
                      <div className="message-sources">
                        <small>Sources:</small>
                        <ul>
                          {message.sources.slice(0, 3).map((source, idx) => (
                            <li key={idx}>
                              <strong>{source.chapterTitle}</strong> in <em>{source.module}</em>
                            </li>
                          ))}
                        </ul>
                      </div>
                    )}

                    {message.isError && (
                      <div className="error-message">
                        <small>Please try rephrasing your question or consult the textbook directly.</small>
                      </div>
                    )}
                  </div>
                  <div className="message-timestamp">{message.timestamp}</div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="chatbot-message bot-message">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className="chatbot-input-form">
            <textarea
              ref={inputRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask a question about the textbook..."
              disabled={isLoading}
              rows="1"
              className="chatbot-input"
            />
            <button
              type="submit"
              disabled={!inputValue.trim() || isLoading}
              className="chatbot-send-button"
              aria-label="Send message"
            >
              <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
              </svg>
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default ChatbotWidget;