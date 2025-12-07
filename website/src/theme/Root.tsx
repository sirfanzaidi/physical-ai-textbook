import React from 'react';
import ChatBot from '@site/src/components/ChatBot';

// This component wraps the entire Docusaurus app
export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <>
      {children}
      <ChatBot />
    </>
  );
}
