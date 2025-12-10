import React from 'react';
import Layout from '@theme-original/Layout';
import LanguageSwitcher from '../../components/LanguageSwitcher';

export default function LayoutWrapper(props: any): JSX.Element {
  return (
    <>
      <div style={{
        position: 'fixed',
        top: '12px',
        right: '120px',
        zIndex: 50,
      }}>
        <LanguageSwitcher />
      </div>
      <Layout {...props} />
    </>
  );
}
