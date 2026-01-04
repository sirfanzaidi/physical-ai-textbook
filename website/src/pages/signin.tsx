import React from 'react';
import { useHistory } from 'react-router-dom';
import { useAuthContext } from '../context/AuthContext';
import { SigninForm } from '../components/auth/SigninForm';
import { LanguageSelector } from '../components/LanguageSelector';
import styles from './auth.module.css';

/**
 * Signin Page
 * Displays the signin form with language selector
 */
export default function SigninPage() {
  const { isAuthenticated } = useAuthContext();
  const history = useHistory();

  // Redirect if already authenticated
  React.useEffect(() => {
    if (isAuthenticated) {
      history.push('/profile');
    }
  }, [isAuthenticated, history]);

  return (
    <div className={styles.authContainer}>
      {/* Language Selector */}
      <div style={{ position: 'absolute', top: '20px', right: '20px', zIndex: 10 }}>
        <LanguageSelector />
      </div>

      {/* Signin Form Card */}
      <div className={styles.authCard}>
        <SigninForm
          redirectUrl="/profile"
          apiBaseUrl="http://localhost:8000"
        />
      </div>
    </div>
  );
}
