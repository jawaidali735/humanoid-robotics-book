import React from 'react';
import ErrorSafeChatbotWidget from '../components/ChatbotWidget/ErrorSafeChatbotWidget';

// Root component with floating chat widget
export default function Root({ children }) {
  return (
    <>
      {children}
      <ErrorSafeChatbotWidget />
    </>
  );
}