import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Layout from '@theme/Layout';

/**
 * Signup Page (Client-Side Only)
 */
export default function SignupPage() {
  return (
    <Layout title="Sign Up" description="Create your account">
      <BrowserOnly fallback={<div style={{ padding: '2rem', textAlign: 'center' }}>Loading...</div>}>
        {() => {
          const { MultiStepSignupForm } = require('../components/auth/MultiStepSignupForm');
          const styles = require('./auth.module.css');

          return (
            <div className={styles.authContainer}>
              <div className={styles.authCard}>
                <MultiStepSignupForm
                  redirectUrl="/profile"
                  apiBaseUrl="http://localhost:8001"
                />
              </div>
            </div>
          );
        }}
      </BrowserOnly>
    </Layout>
  );
}
