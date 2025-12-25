import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import Link from '@docusaurus/Link';

// Navbar with static login/register links (no auth state management)
export default function NavbarWrapper(props) {
  return (
    <>
      <OriginalNavbar {...props} />
      <div
        style={{
          position: 'fixed',
          right: '10rem',
          top: '1rem', // Adjusted position to be relative to viewport instead of navbar height
          zIndex: 1001, // Higher than navbar
          display: 'flex',
          alignItems: 'center'
        }}
      >
        <div className="navbar__item">
          <div className="auth-buttons">
            <Link
              to="/login"
              className="button button--secondary button--sm"
              style={{ marginRight: '8px' }}
            >
              Log in
            </Link>
            <Link
              to="/register"
              className="button button--primary button--sm"
            >
              Sign up
            </Link>
          </div>
        </div>
      </div>
    </>
  );
}