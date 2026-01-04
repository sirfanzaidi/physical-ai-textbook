import React from 'react';
import { I18nextProvider } from 'react-i18next';
import { AuthProvider } from '@site/src/context/AuthContext';
import i18n from '@site/src/i18n/config';

// This component wraps the entire Docusaurus app
export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <I18nextProvider i18n={i18n}>
      <AuthProvider>
        {children}
      </AuthProvider>
    </I18nextProvider>
  );
}