import React, { createContext, useContext, useEffect, useState } from 'react';
import axios from 'axios';
import { API_BASE_URL } from '../lib/apiConfig';

export interface UserProfile {
  id: string;
  email: string;
  name: string;
  programmingBackgrounds: string[];
  frameworksKnown: string[];
  hardwareExperience: string[];
  roboticsInterest: string;
  experience_level: 'beginner' | 'intermediate' | 'advanced';
  completed_onboarding: boolean;
}

interface AuthContextType {
  user: UserProfile | null;
  isLoading: boolean;
  isAuthenticated: boolean;
  updateProfile: (profile: Partial<UserProfile>) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<UserProfile | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // Check if user is already authenticated
    const checkAuth = async () => {
      try {
        const token = localStorage.getItem('auth_token');
        if (!token) {
          setUser(null);
          setIsLoading(false);
          return;
        }

        const apiUrl = typeof window !== 'undefined' && (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1') ? 'http://localhost:8001' : '/api';
        const response = await axios.get(`${apiUrl}/auth/session`, {
          headers: {
            Authorization: `Bearer ${token}`,
          },
        });
        if (response.data.user) {
          setUser(response.data.user);
        }
      } catch (error) {
        // User not authenticated
        localStorage.removeItem('auth_token');
        setUser(null);
      } finally {
        setIsLoading(false);
      }
    };

    checkAuth();
  }, []);

  const updateProfile = async (profile: Partial<UserProfile>) => {
    try {
      const token = localStorage.getItem('auth_token');
      const apiUrl = typeof window !== 'undefined' && (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1') ? 'http://localhost:8001' : '/api';
      const response = await axios.put(`${apiUrl}/users/profile`, profile, {
        headers: {
          Authorization: `Bearer ${token}`,
        },
      });
      setUser(response.data.user);
    } catch (error) {
      console.error('Failed to update profile:', error);
      throw error;
    }
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        isLoading,
        isAuthenticated: !!user,
        updateProfile,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};

export const useAuthContext = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuthContext must be used within AuthProvider');
  }
  return context;
};
