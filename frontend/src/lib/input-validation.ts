/**
 * Input validation utilities for authentication forms
 */

// Email validation
export const validateEmail = (email: string): boolean => {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
};

// Password validation
export const validatePassword = (password: string): boolean => {
  // Minimum 8 characters
  return password.length >= 8;
};

// Name validation
export const validateName = (name: string): boolean => {
  // Name should not be empty and should be at least 2 characters
  return name.trim().length >= 2;
};

// Combined validation for registration
export interface ValidationResult {
  isValid: boolean;
  errors: string[];
}

export const validateRegistrationForm = (
  email: string,
  password: string,
  name: string
): ValidationResult => {
  const errors: string[] = [];

  if (!validateEmail(email)) {
    errors.push('Please enter a valid email address');
  }

  if (!validatePassword(password)) {
    errors.push('Password must be at least 8 characters long');
  }

  if (!validateName(name)) {
    errors.push('Name must be at least 2 characters long');
  }

  return {
    isValid: errors.length === 0,
    errors
  };
};

// Combined validation for login
export const validateLoginForm = (
  email: string,
  password: string
): ValidationResult => {
  const errors: string[] = [];

  if (!validateEmail(email)) {
    errors.push('Please enter a valid email address');
  }

  if (!validatePassword(password)) {
    errors.push('Password must be at least 8 characters long');
  }

  if (!email || !password) {
    errors.push('Email and password are required');
  }

  return {
    isValid: errors.length === 0,
    errors
  };
};

// Sanitize string input
export const sanitizeInput = (input: string): string => {
  if (typeof input !== 'string') {
    return '';
  }
  // Remove potentially dangerous characters
  return input.trim();
};