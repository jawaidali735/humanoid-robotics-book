import React from 'react';
import Layout from '@theme/Layout';

export default function RegisterPage() {
  return (
    <Layout title="Register" description="Create a new account">
      <div style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        minHeight: '80vh',
        padding: '2rem'
      }}>
        <div style={{
          textAlign: 'center',
          padding: '2rem',
          border: '1px solid #ddd',
          borderRadius: '8px',
          boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
          backgroundColor: 'white'
        }}>
          <h1>Sign Up</h1>
          <p>Working on it...</p>
        </div>
      </div>
    </Layout>
  );
}