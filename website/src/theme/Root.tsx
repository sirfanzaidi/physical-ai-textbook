import React from 'react';
import ChatBot from '@site/src/components/ChatBot';
import { AuthProvider } from '@site/src/context/AuthContext';

// This component wraps the entire Docusaurus app
export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <AuthProvider>
      {children}
      <ChatBot />
    </AuthProvider>
  );
}
