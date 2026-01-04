import React from 'react';
import { AuthProvider } from '@site/src/context/AuthContext';
import BrowserOnly from '@docusaurus/BrowserOnly';

// This component wraps the entire Docusaurus app
export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <BrowserOnly fallback={<div>{children}</div>}>
      {() => {
        // Only load i18n on client side to avoid SSR issues
        const { I18nextProvider } = require('react-i18next');
        const i18n = require('@site/src/i18n/config').default;

        return (
          <I18nextProvider i18n={i18n}>
            <AuthProvider>
              {children}
            </AuthProvider>
          </I18nextProvider>
        );
      }}
    </BrowserOnly>
  );
}