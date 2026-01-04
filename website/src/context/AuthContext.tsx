import React, { createContext, useContext, useState, useEffect } from 'react';

interface User {
  id: string;
  email: string;
  name: string;
  experience_level: 'beginner' | 'intermediate' | 'advanced';
  programmingBackgrounds: string[];
  frameworksKnown: string[];
  hardwareExperience: string[];
  roboticsInterest: string;
}

interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (token: string, user: User) => void;
  logout: () => void;
  updateProfile: (data: Partial<User>) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  // Always authenticated as anonymous user
  const user: User = {
    id: 'anonymous',
    email: 'guest@example.com',
    name: 'Guest User',
    experience_level: 'beginner',
    programmingBackgrounds: [],
    frameworksKnown: [],
    hardwareExperience: [],
    roboticsInterest: 'I am interested in everything!',
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        isAuthenticated: true,
        isLoading: false,
        login: () => {},
        logout: () => {},
        updateProfile: async () => { console.log('Update profile mocked'); },
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};

export const useAuthContext = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuthContext must be used within an AuthProvider');
  }
  return context;
};
