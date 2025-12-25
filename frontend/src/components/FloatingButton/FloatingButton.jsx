import React from 'react';
import clsx from 'clsx';
import './FloatingButton.css';

/**
 * FloatingButton component
 * Modern animated floating button for chat widget
 *
 * Props:
 * - onClick: function when button is clicked
 * - label: accessibility label / tooltip
 * - isActive: highlight active state
 * - showNotification: show pulse notification below button
 * - icon: JSX icon (SVG or emoji)
 */
const FloatingButton = ({
  onClick,
  label = "Chat with AI Assistant",
  isActive = false,
  showNotification = false,
  icon
}) => {
  return (
    <button
      className={clsx(
        'floating-button',
        {
          'floating-button--active': isActive,
          'floating-button--notification': showNotification
        }
      )}
      onClick={onClick}
      aria-label={label}
      title={label}
    >
      {/* Icon inside button */}
      {icon ? (
        <span className="floating-button__icon">{icon}</span>
      ) : (
        <svg
          className="floating-button__icon"
          viewBox="0 0 24 24"
          stroke="white"
          strokeWidth="2"
          fill="none"
        >
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
      )}

      {/* Optional notification pulse */}
      {showNotification && (
        <span className="floating-button__notification-dot" aria-hidden="true"></span>
      )}
    </button>
  );
};

export default FloatingButton;
