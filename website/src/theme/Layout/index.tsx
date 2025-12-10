import React from 'react';
import Layout from '@theme-original/Layout';
import LanguageSwitcher from '@site/src/components/LanguageSwitcher';
import type LayoutType from '@theme/Layout';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): JSX.Element {
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
