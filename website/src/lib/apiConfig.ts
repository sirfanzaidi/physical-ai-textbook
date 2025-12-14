/**
 * API Configuration utility
 * Handles API base URL for both development and production
 */

export function getApiUrl(): string {
  // For development, use the backend on port 8000
  // For production, use Railway deployed backend
  if (typeof window !== 'undefined') {
    const isDevelopment = window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1';
    if (isDevelopment) {
      return 'http://localhost:8000';
    }
  }

  // For production, use Railway backend
  return 'https://physical-ai-textbook-production.up.railway.app';
}

export const API_BASE_URL = getApiUrl();
