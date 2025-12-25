/**
 * API utilities for chatbot integration
 */

// Get the backend API URL from environment variable or default to empty string (for relative paths)
const BACKEND_API_BASE = process.env.REACT_APP_CHATBOT_API_ENDPOINT || '';

/**
 * Send a chat query to the backend API
 */
export const sendChatQuery = async (query, context = {}) => {
  try {
    // Use the backend API URL if available, otherwise use relative path (for proxy)
    const apiUrl = BACKEND_API_BASE ? `${BACKEND_API_BASE}/api/chat` : '/api/chat';

    const response = await fetch(apiUrl, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query,
        session_id: context.sessionId || null,
        context: {
          chapter: context.chapter || null,
          section: context.section || null,
          selected_text: context.selectedText || null,
          ...context.additionalContext
        }
      })
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error sending chat query:', error);
    throw error;
  }
};

/**
 * Send a context-aware query to the backend API
 */
export const sendContextQuery = async (selectedText, query, documentContext = {}) => {
  try {
    const apiUrl = BACKEND_API_BASE ? `${BACKEND_API_BASE}/api/chat/context-query` : '/api/chat/context-query';

    const response = await fetch(apiUrl, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        selected_text: selectedText,
        query,
        document_context: documentContext
      })
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error sending context query:', error);
    throw error;
  }
};

/**
 * Get health status of the chatbot service
 */
export const getChatHealth = async () => {
  try {
    const apiUrl = BACKEND_API_BASE ? `${BACKEND_API_BASE}/api/chat/health` : '/api/chat/health';

    const response = await fetch(apiUrl);

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error getting chat health:', error);
    throw error;
  }
};

/**
 * Get rate limit status
 */
export const getRateLimitStatus = async (ipAddress) => {
  try {
    const apiUrl = BACKEND_API_BASE ? `${BACKEND_API_BASE}/api/chat/rate-limit` : '/api/chat/rate-limit';

    const response = await fetch(apiUrl, {
      method: 'GET',
      headers: {
        'X-Forwarded-For': ipAddress || ''
      }
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error getting rate limit status:', error);
    throw error;
  }
};