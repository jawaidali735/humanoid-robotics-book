import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import FloatingButton from '../FloatingButton/FloatingButton';
import './ChatbotWidget.css';

/**
 * ChatbotWidget component with expandable interface and quick action buttons
 */
const ChatbotWidget = ({
  title = "AI Assistant",
  quickActions = [],
  apiEndpoint = process.env.REACT_APP_CHATBOT_API_ENDPOINT || "http://localhost:8007"
}) => {
  // API functions that use the provided apiEndpoint
  const sendChatQuery = async (query, context = {}) => {
    try {
      const response = await fetch(`${apiEndpoint}/api/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: query,
          session_id: context.sessionId || null
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const responseJson = await response.json();
      return responseJson;
    } catch (error) {
      console.error('Error sending chat query:', error);
      throw error;
    }
  };

  const sendContextQuery = async (selectedText, query, documentContext = {}) => {
    try {
      const response = await fetch(`${apiEndpoint}/api/chat/context-query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          selected_text: selectedText,
          query: query,
          document_context: documentContext,
          session_id: null // Remove problematic selector, using null for now
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const responseJson = await response.json();
      return responseJson;
    } catch (error) {
      console.error('Error sending context query:', error);
      throw error;
    }
  };

  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Get selected text when user makes a selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText.length > 0 && selectedText.length < 1000) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async (messageText = null) => {
    const textToSend = messageText || inputValue.trim();
    if (!textToSend) return;

    const userMessage = {
      id: Date.now(),
      sender: 'user',
      content: textToSend,
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Determine if this is a context-aware query (e.g., explaining selected text)
      const isContextQuery = selectedText;

      let response;
      if (isContextQuery && selectedText) {
        response = await sendContextQuery(selectedText, textToSend, {
          chapter: document.querySelector('[data-chapter]')?.getAttribute('data-chapter') || '',
          section: document.querySelector('[data-section]')?.getAttribute('data-section') || ''
        });
      } else {
        response = await sendChatQuery(textToSend, {
          sessionId: 'session-' + Date.now(), // Generate a simple session ID
          history: messages
        });
      }

      const botMessage = {
        id: Date.now() + 1,
        sender: 'assistant',
        content: response.answer || response.content || 'I received your message',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        sender: 'assistant',
        content: 'Sorry, I encountered an issue processing your request. Please try again.',
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleQuickAction = (action) => {
    if (action.onClick) {
      action.onClick();
    } else if (action.label) {
      setInputValue(action.label);
      setTimeout(() => {
        if (inputRef.current) {
          inputRef.current.focus();
        }
      }, 100);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className="chatbot-widget-wrapper">
      {/* Floating Button */}
      <FloatingButton
        onClick={toggleChat}
        isOpen={isOpen}
        label={isOpen ? "Close chat" : "Open chat"}
      />

      {/* Chat Window */}
      {isOpen && (
        <div className="chatbot-widget-window">
          <div className="chatbot-header">
            <h3>{title}</h3>
            <button
              className="chatbot-close-button"
              onClick={toggleChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="chatbot-welcome">
                <p>Hello! I'm your AI assistant. How can I help you today?</p>
                {quickActions.length > 0 && (
                  <div className="chatbot-quick-actions">
                    {quickActions.map((action, index) => (
                      <button
                        key={index}
                        className="chatbot-quick-action"
                        onClick={() => handleQuickAction(action)}
                      >
                        {action.icon && <span className="quick-action-icon">{action.icon}</span>}
                        {action.label}
                      </button>
                    ))}
                  </div>
                )}
              </div>
            ) : (
              <div className="chatbot-conversation">
                {messages.map((message) => (
                  <div
                    key={message.id}
                    className={clsx('chatbot-message', `chatbot-message--${message.sender}`)}
                  >
                    <div className="chatbot-message-content">
                      {message.content}
                    </div>
                    <div className="chatbot-message-timestamp">
                      {new Date(message.timestamp).toLocaleTimeString([], {
                        hour: '2-digit',
                        minute: '2-digit'
                      })}
                    </div>
                  </div>
                ))}
                {isLoading && (
                  <div className="chatbot-message chatbot-message--assistant">
                    <div className="chatbot-message-content">
                      <div className="chatbot-typing-indicator">
                        <span></span>
                        <span></span>
                        <span></span>
                      </div>
                    </div>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>
            )}
          </div>

          <div className="chatbot-input-area">
            {selectedText && (
              <div className="chatbot-selected-text-preview">
                <strong>Selected text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
                <button
                  className="chatbot-clear-selection"
                  onClick={() => setSelectedText('')}
                  title="Clear selection"
                >
                  ×
                </button>
              </div>
            )}
            <div className="chatbot-input-controls">
              <textarea
                ref={inputRef}
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder="Type your message here..."
                className="chatbot-input"
                rows="1"
                disabled={isLoading}
              />
              <button
                onClick={() => sendMessage()}
                disabled={!inputValue.trim() || isLoading}
                className="chatbot-send-button"
                aria-label="Send message"
              >
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatbotWidget;