import React from 'react';

// Safe wrapper for ChatbotWidget that handles authentication context errors
const SafeChatbotWidget = (props) => {
  try {
    // Dynamically import and render the actual ChatbotWidget
    const ChatbotWidget = require('./ChatbotWidget.jsx').default;
    return <ChatbotWidget {...props} />;
  } catch (error) {
    // If there's an error (like missing auth context), return a minimal component
    console.warn('ChatbotWidget could not be rendered:', error.message);
    return null; // Return nothing if there's an error
  }
};

export default SafeChatbotWidget;