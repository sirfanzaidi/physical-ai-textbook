import React, { createContext, useContext, useEffect, useState } from 'react';
import axios from 'axios';

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
        const response = await axios.get('/api/auth/session');
        if (response.data.user) {
          setUser(response.data.user);
        }
      } catch (error) {
        // User not authenticated
        setUser(null);
      } finally {
        setIsLoading(false);
      }
    };

    checkAuth();
  }, []);

  const updateProfile = async (profile: Partial<UserProfile>) => {
    try {
      const response = await axios.put('/api/users/profile', profile);
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
