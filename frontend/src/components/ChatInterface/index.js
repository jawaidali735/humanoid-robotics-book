import React, { useState, useRef, useEffect } from 'react';
import './ChatInterface.css';

/**
 * ChatInterface component for humanoid robotics Q&A
 * Provides a chat interface that connects to the FastAPI backend
 */
const ChatInterface = ({ apiEndpoint = process.env.REACT_APP_CHATBOT_API_ENDPOINT || 'http://localhost:8007' }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messagesEndRef = useRef(null);

  // Scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Handle sending a message
  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // Call the backend API - using the correct endpoint
      const response = await fetch(`${apiEndpoint}/api/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,  // Changed from 'message' to 'query' to match backend expectation
          session_id: localStorage.getItem('chatSessionId') || null
        })
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const responseJson = await response.json();

      // Check if the response is successful
      if (!responseJson.success) {
        // Don't throw an error for business logic failures, just show the error response
        const botMessage = {
          id: Date.now() + 1,
          text: responseJson.data?.answer || responseJson.error?.message || 'API request failed',
          sender: 'bot',
          sources: responseJson.data?.sources || [],
          timestamp: responseJson.data?.timestamp || new Date().toISOString()
        };
        setMessages(prev => [...prev, botMessage]);

        // Store session ID if provided
        if (responseJson.data?.response_id) {
          localStorage.setItem('chatSessionId', responseJson.data.response_id);
        }
        return; // Exit early since we've handled the error response
      }

      // Add bot response to chat - using the correct field names from backend
      const botMessage = {
        id: Date.now() + 1,
        text: responseJson.data.answer,
        sender: 'bot',
        sources: responseJson.data.sources || [],
        timestamp: responseJson.data.timestamp
      };

      setMessages(prev => [...prev, botMessage]);

      // Store session ID if provided
      if (responseJson.data.response_id) {
        localStorage.setItem('chatSessionId', responseJson.data.response_id);
      }
    } catch (err) {
      console.error('Chat error:', err);

      // Check if it's a network/connection error vs processing error
      const isConnectionError = err.name === 'TypeError' ||
                               err.message.includes('fetch') ||
                               err.message.includes('network') ||
                               err.message.includes('Failed to fetch') ||
                               err.status === 0;

      if (isConnectionError) {
        setError(err.message);
        // Add connection error message to chat
        const errorMessage = {
          id: Date.now() + 1,
          text: `Connection error: ${err.message}. Please check your connection to the API.`,
          sender: 'system',
          isError: true,
          timestamp: new Date().toISOString()
        };
        setMessages(prev => [...prev, errorMessage]);
      } else {
        // For processing errors, add a more user-friendly message from the API if available
        const errorMessage = {
          id: Date.now() + 1,
          text: err.message || 'The AI service is having trouble processing your request, but it is still connected. Please try again.',
          sender: 'system',
          isError: false, // Not a connection error
          timestamp: new Date().toISOString()
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } finally {
      setIsLoading(false);
    }
  };

  // Handle key press (Enter to send)
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Handle text selection for contextual queries
  const handleTextSelection = () => {
    const selectedText = window.getSelection().toString().trim();
    if (selectedText) {
      // You could implement contextual queries here
      // For now, we'll just add the selected text to the input
      setInputValue(prev => prev + (prev ? ' ' : '') + selectedText);
    }
  };

  return (
    <div className="chat-interface">
      <div className="chat-header">
        <h3>Humanoid Robotics Assistant</h3>
        <p>Ask questions about humanoid robotics concepts</p>
      </div>

      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your humanoid robotics assistant. Ask me anything about humanoid robotics, kinematics, control systems, or related topics.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.sender}-message`}
            >
              <div className="message-content">
                <p>{message.text}</p>

                {message.sources && message.sources.length > 0 && (
                  <div className="sources">
                    <h4>Sources:</h4>
                    <ul>
                      {message.sources.map((source, idx) => (
                        <li key={idx} className="source-item">
                          <p>{source.substring(0, 150)}{source.length > 150 ? '...' : ''}</p>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            </div>
          ))
        )}

        {isLoading && (
          <div className="message bot-message">
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

      {error && (
        <div className="error-message">
          <p>Error: {error}</p>
        </div>
      )}

      <div className="chat-input-area">
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about humanoid robotics..."
          rows="3"
          disabled={isLoading}
        />
        <button
          onClick={handleSendMessage}
          disabled={!inputValue.trim() || isLoading}
          className="send-button"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>

      <div className="chat-footer">
        <p>Powered by AI with source attribution from the humanoid robotics book</p>
      </div>
    </div>
  );
};

export default ChatInterface;