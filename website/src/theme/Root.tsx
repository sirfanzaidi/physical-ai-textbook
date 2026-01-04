import React from 'react';
import { AuthProvider } from '@site/src/context/AuthContext';

// This component wraps the entire Docusaurus app
// Note: i18n temporarily disabled for build compatibility
export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  );
}