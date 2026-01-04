import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Layout from '@theme/Layout';

/**
 * Signin Page (Client-Side Only)
 */
export default function SigninPage() {
  return (
    <Layout title="Sign In" description="Sign in to your account">
      <BrowserOnly fallback={<div style={{ padding: '2rem', textAlign: 'center' }}>Loading...</div>}>
        {() => {
          const { SigninForm } = require('../components/auth/SigninForm');
          const styles = require('./auth.module.css');

          return (
            <div className={styles.authContainer}>
              <div className={styles.authCard}>
                <SigninForm
                  redirectUrl="/profile"
                  apiBaseUrl="http://localhost:8000"
                />
              </div>
            </div>
          );
        }}
      </BrowserOnly>
    </Layout>
  );
}
