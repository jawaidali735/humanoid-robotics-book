/**
 * Accessibility utilities for Futuristic Book Website UI
 * WCAG 2.1 AA compliance utilities and helpers
 */

/**
 * Focus trap utility for modal dialogs and expanded components
 */
export class FocusTrap {
  constructor(element) {
    this.element = element;
    this.focusableElements = this.getFocusableElements();
    this.firstFocusableElement = this.focusableElements[0];
    this.lastFocusableElement = this.focusableElements[this.focusableElements.length - 1];
  }

  getFocusableElements() {
    const focusableSelectors = [
      'a[href]',
      'button:not([disabled])',
      'input:not([disabled])',
      'select:not([disabled])',
      'textarea:not([disabled])',
      '[tabindex]:not([tabindex="-1"])'
    ].join(', ');

    return Array.from(this.element.querySelectorAll(focusableSelectors))
      .filter(el => !el.hasAttribute('disabled') && !el.getAttribute('aria-hidden'));
  }

  trap(event) {
    if (this.focusableElements.length === 0) return;

    if (event.key === 'Tab') {
      if (event.shiftKey) {
        if (document.activeElement === this.firstFocusableElement) {
          event.preventDefault();
          this.lastFocusableElement.focus();
        }
      } else {
        if (document.activeElement === this.lastFocusableElement) {
          event.preventDefault();
          this.firstFocusableElement.focus();
        }
      }
    }
  }

  enable() {
    this.element.addEventListener('keydown', this.trap.bind(this));
  }

  disable() {
    this.element.removeEventListener('keydown', this.trap.bind(this));
  }
}

/**
 * ARIA live region utility for announcing dynamic content changes
 */
export class AriaLive {
  constructor() {
    this.container = document.createElement('div');
    this.container.setAttribute('aria-live', 'polite');
    this.container.setAttribute('aria-atomic', 'true');
    this.container.className = 'sr-only';
    this.container.style.position = 'absolute';
    this.container.style.left = '-9999px';
    this.container.style.top = 'auto';
    this.container.style.width = '1px';
    this.container.style.height = '1px';
    this.container.style.overflow = 'hidden';

    document.body.appendChild(this.container);
  }

  announce(message) {
    this.container.textContent = message;

    // Force reflow to ensure the announcement is read
    this.container.style.clip = 'rect(0 0 0 0)';
    this.container.style.clip = 'auto';
  }
}

/**
 * Skip link utility for keyboard navigation
 */
export function initSkipLinks() {
  const skipLink = document.createElement('a');
  skipLink.href = '#main-content';
  skipLink.textContent = 'Skip to main content';
  skipLink.className = 'skip-link';
  skipLink.style.position = 'absolute';
  skipLink.style.left = '-9999px';
  skipLink.style.top = 'auto';
  skipLink.style.width = '1px';
  skipLink.style.height = '1px';
  skipLink.style.overflow = 'hidden';
  skipLink.style.backgroundColor = 'var(--color-accent-primary, #00ffff)';
  skipLink.style.color = 'var(--color-background, #0a0a0a)';
  skipLink.style.padding = 'var(--spacing-2)';
  skipLink.style.borderRadius = 'var(--border-radius-md)';

  skipLink.addEventListener('focus', () => {
    skipLink.style.left = 'var(--spacing-4)';
    skipLink.style.top = 'var(--spacing-4)';
    skipLink.style.width = 'auto';
    skipLink.style.height = 'auto';
  });

  skipLink.addEventListener('blur', () => {
    skipLink.style.left = '-9999px';
  });

  document.body.insertBefore(skipLink, document.body.firstChild);
}

/**
 * Screen reader only utility class
 */
export function srOnly(element) {
  element.style.position = 'absolute';
  element.style.left = '-9999px';
  element.style.top = 'auto';
  element.style.width = '1px';
  element.style.height = '1px';
  element.style.overflow = 'hidden';
}

/**
 * High contrast mode detection
 */
export function isHighContrastMode() {
  const style = window.getComputedStyle(document.body, null);
  const bgColor = style.backgroundColor;
  const color = style.color;

  // If background and text colors are the same, likely in high contrast mode
  return bgColor === color;
}

/**
 * Keyboard navigation utilities
 */
export class KeyboardNavigation {
  static init() {
    // Add class to body when using keyboard navigation
    document.body.addEventListener('keydown', (e) => {
      if (e.key === 'Tab') {
        document.body.classList.add('keyboard-navigation');
      }
    });

    document.body.addEventListener('mousedown', () => {
      document.body.classList.remove('keyboard-navigation');
    });
  }
}

/**
 * Focus visible polyfill for better focus indication
 */
export function initFocusVisible() {
  // Add focus-visible class when using keyboard navigation
  let hadKeyboardEvent = true;

  const inputTypesWhitelist = {
    text: true,
    search: true,
    url: true,
    tel: true,
    email: true,
    password: true,
    number: true,
    date: true,
    month: true,
    week: true,
    time: true,
    datetime: true,
    'datetime-local': true,
  };

  function isValidKeyTarget(event) {
    if (event.target && event.target.nodeName && event.target.nodeName.toLowerCase() === 'input') {
      return inputTypesWhitelist[event.target.type] || !event.target.type;
    }
    return true;
  }

  function onInitialPointerMove(event) {
    if (event.type === 'mousedown') {
      hadKeyboardEvent = false;
    }
  }

  document.addEventListener('keydown', (event) => {
    if (isValidKeyTarget(event)) {
      hadKeyboardEvent = true;
    }
  }, true);

  document.addEventListener('mousedown', onInitialPointerMove, true);
  document.addEventListener('pointermove', onInitialPointerMove, true);
  document.addEventListener('touchstart', onInitialPointerMove, true);

  // Add focus-visible class to elements when appropriate
  document.addEventListener('focusin', (event) => {
    if (hadKeyboardEvent) {
      event.target.classList.add('focus-visible');
    }
  });

  document.addEventListener('focusout', (event) => {
    event.target.classList.remove('focus-visible');
  });
}

/**
 * Color contrast checker utility
 */
export function checkColorContrast(foregroundColor, backgroundColor) {
  // Convert hex to RGB if needed
  const fg = hexToRgb(foregroundColor) || foregroundColor;
  const bg = hexToRgb(backgroundColor) || backgroundColor;

  const fgLuminance = getRelativeLuminance(fg);
  const bgLuminance = getRelativeLuminance(bg);

  const contrastRatio = (Math.max(fgLuminance, bgLuminance) + 0.05) /
                       (Math.min(fgLuminance, bgLuminance) + 0.05);

  return {
    ratio: parseFloat(contrastRatio.toFixed(2)),
    passesAA: contrastRatio >= 4.5, // Minimum for normal text
    passesAAA: contrastRatio >= 7.0, // Enhanced contrast
    passesLargeAA: contrastRatio >= 3.0 // Minimum for large text (18pt+ or 14pt+ bold)
  };
}

function hexToRgb(hex) {
  const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return result ? {
    r: parseInt(result[1], 16),
    g: parseInt(result[2], 16),
    b: parseInt(result[3], 16)
  } : null;
}

function getRelativeLuminance(color) {
  if (!color) return 0;

  let r = color.r / 255;
  let g = color.g / 255;
  let b = color.b / 255;

  r = (r <= 0.03928) ? r / 12.92 : Math.pow((r + 0.055) / 1.055, 2.4);
  g = (g <= 0.03928) ? g / 12.92 : Math.pow((g + 0.055) / 1.055, 2.4);
  b = (b <= 0.03928) ? b / 12.92 : Math.pow((b + 0.055) / 1.055, 2.4);

  return 0.2126 * r + 0.7152 * g + 0.0722 * b;
}

/**
 * Initialize all accessibility features
 */
export function initAccessibility() {
  // Initialize keyboard navigation detection
  KeyboardNavigation.init();

  // Initialize focus-visible polyfill
  initFocusVisible();

  // Initialize skip links
  initSkipLinks();

  // Add necessary CSS for focus indicators
  const style = document.createElement('style');
  style.textContent = `
    .focus-visible {
      outline: 2px solid var(--color-focus, #00ffff) !important;
      outline-offset: 2px;
    }

    .keyboard-navigation :focus:not(.focus-visible) {
      outline: none;
    }

    .skip-link {
      position: absolute;
      left: -9999px;
      top: auto;
      width: 1px;
      height: 1px;
      overflow: hidden;
      background: var(--color-accent-primary, #00ffff);
      color: var(--color-background, #0a0a0a);
      padding: var(--spacing-2);
      border-radius: var(--border-radius-md);
      z-index: var(--z-tooltip);
    }

    .skip-link:focus {
      left: var(--spacing-4);
      top: var(--spacing-4);
      width: auto;
      height: auto;
    }
  `;

  document.head.appendChild(style);
}

// Initialize accessibility features when DOM is loaded
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initAccessibility);
} else {
  initAccessibility();
}