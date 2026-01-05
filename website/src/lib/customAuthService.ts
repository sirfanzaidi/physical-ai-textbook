/**
 * Custom Authentication Service
 * Direct API calls to our FastAPI backend
 */

const API_BASE_URL = 'http://localhost:8001';

export interface SignUpData {
  email: string;
  password: string;
  name: string;
  programming_backgrounds?: string[];
  frameworks_known?: string[];
  hardware_experience?: string[];
  robotics_interest?: string;
  experience_level?: string;
}

export interface SignInData {
  email: string;
  password: string;
}

export interface AuthResponse {
  user: {
    id: string;
    email: string;
    name: string;
    programming_backgrounds: string[];
    frameworks_known: string[];
    hardware_experience: string[];
    robotics_interest: string | null;
    experience_level: string;
    completed_onboarding: boolean;
    created_at: string;
    updated_at: string;
  };
  token: string;
  expires_at: string;
}

export interface SessionResponse {
  user: {
    id: string;
    email: string;
    name: string;
    programming_backgrounds: string[];
    frameworks_known: string[];
    hardware_experience: string[];
    robotics_interest: string | null;
    experience_level: string;
    completed_onboarding: boolean;
    created_at: string;
    updated_at: string;
  };
}

/**
 * Sign up a new user
 */
export async function customSignUp(data: SignUpData): Promise<AuthResponse> {
  const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(data),
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail?.error || 'Signup failed');
  }

  const result = await response.json();

  // Store token in localStorage
  localStorage.setItem('auth_token', result.token);
  localStorage.setItem('user', JSON.stringify(result.user));

  return result;
}

/**
 * Sign in an existing user
 */
export async function customSignIn(data: SignInData): Promise<AuthResponse> {
  const response = await fetch(`${API_BASE_URL}/api/auth/signin`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(data),
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail?.error || 'Sign in failed');
  }

  const result = await response.json();

  // Store token in localStorage
  localStorage.setItem('auth_token', result.token);
  localStorage.setItem('user', JSON.stringify(result.user));

  return result;
}

/**
 * Sign out the current user
 */
export async function customSignOut(): Promise<void> {
  const token = localStorage.getItem('auth_token');

  if (token) {
    try {
      await fetch(`${API_BASE_URL}/api/auth/signout`, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });
    } catch (error) {
      console.error('Signout error:', error);
    }
  }

  // Clear local storage
  localStorage.removeItem('auth_token');
  localStorage.removeItem('user');
}

/**
 * Get current user session
 */
export async function getSession(): Promise<SessionResponse | null> {
  const token = localStorage.getItem('auth_token');

  if (!token) {
    return null;
  }

  try {
    const response = await fetch(`${API_BASE_URL}/api/auth/session`, {
      headers: {
        'Authorization': `Bearer ${token}`,
      },
    });

    if (!response.ok) {
      // Token is invalid, clear it
      localStorage.removeItem('auth_token');
      localStorage.removeItem('user');
      return null;
    }

    return await response.json();
  } catch (error) {
    console.error('Session error:', error);
    return null;
  }
}

/**
 * Check if user is authenticated
 */
export function isAuthenticated(): boolean {
  return !!localStorage.getItem('auth_token');
}

/**
 * Get current user from local storage
 */
export function getCurrentUser() {
  const userStr = localStorage.getItem('user');
  return userStr ? JSON.parse(userStr) : null;
}
