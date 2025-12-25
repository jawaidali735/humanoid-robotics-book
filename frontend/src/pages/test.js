import React from 'react';
import Layout from '@theme/Layout';

export default function TestPage() {
  return (
    <Layout title="Test Page" description="Test page to check if basic pages work">
      <main>
        <div style={{ padding: '2rem', textAlign: 'center' }}>
          <h1>Test Page</h1>
          <p>If you see this, basic page rendering works.</p>
          <p>There might be an issue with the homepage specifically.</p>
        </div>
      </main>
    </Layout>
  );
}