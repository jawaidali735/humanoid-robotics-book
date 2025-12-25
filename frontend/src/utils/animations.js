/**
 * Animation utilities for Futuristic Book Website UI
 * Performance-optimized animation functions
 */

/**
 * Smooth scroll to element with easing function
 */
export const smoothScrollTo = (element, duration = 500) => {
  if (!element) return;

  const start = element.scrollTop;
  const target = element.offsetTop;
  const distance = target - start;
  const startTime = performance.now();

  const easeInOutCubic = (t) => {
    return t < 0.5 ? 4 * t * t * t : (t - 1) * (2 * t - 2) * (2 * t - 2) + 1;
  };

  const animateScroll = (currentTime) => {
    const elapsed = currentTime - startTime;
    const progress = Math.min(elapsed / duration, 1);
    const ease = easeInOutCubic(progress);

    element.scrollTop = start + distance * ease;

    if (progress < 1) {
      requestAnimationFrame(animateScroll);
    }
  };

  requestAnimationFrame(animateScroll);
};

/**
 * Scroll to element with polyfill for native smooth scrolling
 */
export const scrollToElement = (selector, options = {}) => {
  const element = typeof selector === 'string' ? document.querySelector(selector) : selector;
  if (!element) return;

  const defaultOptions = {
    behavior: 'smooth',
    block: 'start',
    inline: 'nearest'
  };

  const scrollOptions = { ...defaultOptions, ...options };

  // Try native smooth scrolling first
  if ('scrollBehavior' in document.documentElement.style) {
    element.scrollIntoView(scrollOptions);
  } else {
    // Fallback for older browsers
    const targetPosition = element.getBoundingClientRect().top + window.pageYOffset;
    window.scrollTo({
      top: targetPosition,
      behavior: 'auto'
    });
  }
};

/**
 * Animate element into view with intersection observer
 */
export const animateOnScroll = (selector, animationClass = 'animate-fade-in') => {
  const elements = typeof selector === 'string'
    ? document.querySelectorAll(selector)
    : Array.isArray(selector) ? selector : [selector];

  const observerOptions = {
    threshold: 0.1,
    rootMargin: '0px 0px -50px 0px'
  };

  const observer = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
      if (entry.isIntersecting) {
        entry.target.classList.add(animationClass);
        observer.unobserve(entry.target);
      }
    });
  }, observerOptions);

  elements.forEach(element => {
    if (element) {
      observer.observe(element);
    }
  });

  // Cleanup function
  return () => {
    elements.forEach(element => {
      if (element) {
        observer.unobserve(element);
      }
    });
  };
};

/**
 * Debounce function for performance optimization
 */
export const debounce = (func, wait) => {
  let timeout;
  return function executedFunction(...args) {
    const later = () => {
      clearTimeout(timeout);
      func(...args);
    };
    clearTimeout(timeout);
    timeout = setTimeout(later, wait);
  };
};

/**
 * Throttle function for performance optimization
 */
export const throttle = (func, limit) => {
  let inThrottle;
  return function() {
    const args = arguments;
    const context = this;
    if (!inThrottle) {
      func.apply(context, args);
      inThrottle = true;
      setTimeout(() => inThrottle = false, limit);
    }
  };
};

/**
 * Performance-optimized animation frame helper
 */
export class AnimationFrameHelper {
  constructor() {
    this.animationFrameId = null;
    this.isRunning = false;
  }

  start(callback) {
    if (this.isRunning) return;

    this.isRunning = true;
    const animate = () => {
      callback();
      if (this.isRunning) {
        this.animationFrameId = requestAnimationFrame(animate);
      }
    };
    animate();
  }

  stop() {
    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
      this.animationFrameId = null;
    }
    this.isRunning = false;
  }
}

/**
 * CSS variable animation helper for performance
 */
export const animateCSSVariable = (element, property, from, to, duration = 300) => {
  return new Promise((resolve) => {
    element.style.setProperty(property, from);

    // Force reflow
    element.offsetHeight;

    element.style.setProperty(property, to);
    element.style.transition = `all ${duration}ms ease-in-out`;

    setTimeout(() => {
      element.style.transition = '';
      resolve();
    }, duration);
  });
};

/**
 * Parallax effect for hero elements
 */
export const initParallax = (selector, speedFactor = 0.5) => {
  const elements = document.querySelectorAll(selector);

  const handleScroll = () => {
    const scrolled = window.pageYOffset;
    elements.forEach(element => {
      const rate = element.getAttribute('data-parallax-speed') || speedFactor;
      const yPos = -(scrolled * rate);
      element.style.transform = `translate3d(0, ${yPos}px, 0)`;
    });
  };

  window.addEventListener('scroll', throttle(handleScroll, 16)); // ~60fps

  // Cleanup function
  return () => {
    window.removeEventListener('scroll', handleScroll);
  };
};

/**
 * Staggered animation for lists or grids
 */
export const staggerAnimation = (selector, delay = 100, animationClass = 'animate-fade-in') => {
  const elements = document.querySelectorAll(selector);

  elements.forEach((element, index) => {
    element.style.animationDelay = `${index * delay}ms`;
    element.classList.add(animationClass);
  });
};

/**
 * Initialize all performance-optimized animations
 */
export const initPerformanceAnimations = () => {
  // Initialize scroll animations
  animateOnScroll('[data-animate-on-scroll]');

  // Initialize parallax effects
  initParallax('[data-parallax]', 0.3);

  // Initialize staggered animations
  staggerAnimation('[data-stagger]');
};

// Initialize animations when DOM is loaded
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initPerformanceAnimations);
} else {
  initPerformanceAnimations();
}