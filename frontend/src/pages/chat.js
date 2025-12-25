import React from 'react';
import Layout from '@theme/Layout';
import ChatbotWidget from '@site/src/components/ChatbotWidget/ChatbotWidget';

function ChatPage() {
  return (
    <Layout
      title="Humanoid Robotics Chat"
      description="Ask questions about humanoid robotics concepts and get answers with source attribution">
      <ChatbotWidget />
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <h1>Humanoid Robotics Assistant</h1>
            <p>Ask questions about humanoid robotics, kinematics, control systems, and related topics. Responses include source attribution from the book content.</p>

            <div style={{ height: '650px', marginTop: '20px' }}>
              <div style={{ height: '100%', border: '1px solid #ddd', borderRadius: '8px' }}>
                <p style={{ textAlign: 'center', padding: '20px', color: '#666' }}>
                  The floating chat widget is now available in the bottom-right corner! Click the chat icon to start asking questions about humanoid robotics.
                </p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default ChatPage;