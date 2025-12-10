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

  // Remove Sign In and Sign Up from navbar items if present
  const filteredItems = (props.items || []).filter(
    (item: any) => item.label !== 'Sign In' && item.label !== 'Sign Up'
  );

  return (
    <div style={{ position: 'relative' }}>
      <NavbarDefault
        {...props}
        items={filteredItems}
      />
      {/* Language Switcher - positioned absolutely in top right */}
      <div style={{
        position: 'absolute',
        top: 0,
        right: '200px', // Make room for auth buttons
        height: '60px',
        display: 'flex',
        alignItems: 'center',
        paddingRight: '12px',
        zIndex: 100,
      }}>
        <LanguageSwitcher />
      </div>
      {/* Auth Buttons - positioned absolutely in top right */}
      <div style={{
        position: 'absolute',
        top: 0,
        right: '12px',
        height: '60px',
        display: 'flex',
        alignItems: 'center',
        gap: '12px',
        zIndex: 100,
      }}>
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
    </div>
  );
}
