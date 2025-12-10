import React from 'react';
import { useAuthContext } from '@site/src/context/AuthContext';
import NavbarDefault from '@theme-original/Navbar';
import Link from '@docusaurus/Link';
import LanguageSwitcher from '@site/src/components/LanguageSwitcher';
import styles from './navbar.module.css';

/**
 * Custom Navbar wrapper that adds auth buttons
 */
export default function Navbar(props: any): JSX.Element {
  const { isAuthenticated, user } = useAuthContext();

  return (
    <NavbarDefault
      {...props}
      endItems={[
        ...(props.endItems || []),
        {
          type: 'custom-auth-buttons',
          isAuthenticated,
          user,
        },
      ]}
    >
      <div style={{ display: 'flex', alignItems: 'center', gap: '12px' }}>
        <LanguageSwitcher />
        <div className={styles.authButtons}>
          {!isAuthenticated ? (
            <>
              <Link
                to="/signin"
                className={`${styles.navLink} ${styles.signin}`}
              >
                Sign In
              </Link>
              <Link
                to="/signup"
                className={`${styles.navLink} ${styles.signup}`}
              >
                Sign Up
              </Link>
            </>
          ) : (
            <>
              <Link
                to="/profile"
                className={styles.navLink}
              >
                👤 {user?.name || 'Profile'}
              </Link>
              <button
                className={`${styles.navLink} ${styles.logout}`}
                onClick={() => {
                  localStorage.removeItem('auth_token');
                  window.location.href = '/';
                }}
              >
                Logout
              </button>
            </>
          )}
        </div>
      </div>
    </NavbarDefault>
  );
}
