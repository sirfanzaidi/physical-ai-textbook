import React from 'react';
import Layout from '@theme-original/Layout';
import RAGChat from '@site/src/components/RAGChat';

export default function LayoutWrapper(props: any): JSX.Element {
  return (
    <>
      <Layout {...props} />
      <RAGChat bookId="physical-ai" />
    </>
  );
}
