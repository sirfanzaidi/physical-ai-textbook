import React from 'react';
import { useAuthContext } from '@site/src/context/AuthContext';
import NavbarDefault from '@theme-original/Navbar';
import type NavbarType from '@theme/Navbar';
import type { Props } from '@theme/Navbar';
import Link from '@docusaurus/Link';
import styles from './navbar.module.css';

type NavbarProps = Props;

/**
 * Custom Navbar wrapper that adds auth buttons
 */
export default function Navbar(props: NavbarProps): JSX.Element {
  const { isAuthenticated, user } = useAuthContext();

  return (
    <>
      <NavbarDefault {...props}>
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
      </NavbarDefault>
    </>
  );
}
