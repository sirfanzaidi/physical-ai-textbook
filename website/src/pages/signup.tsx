import React from 'react';
import { useHistory } from 'react-router-dom';
import { useAuthContext } from '../context/AuthContext';
import { MultiStepSignupForm } from '../components/auth/MultiStepSignupForm';
import { LanguageSelector } from '../components/LanguageSelector';
import styles from './auth.module.css';

/**
 * Signup Page
 * Displays the 4-step signup form with language selector
 */
export default function SignupPage() {
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

      {/* Signup Form Card */}
      <div className={styles.authCard}>
        <MultiStepSignupForm
          redirectUrl="/profile"
          apiBaseUrl="http://localhost:8000"
        />
      </div>
    </div>
  );
}
