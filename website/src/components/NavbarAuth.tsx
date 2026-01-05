import React from 'react';
import { useAuthContext } from '../context/AuthContext';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useHistory } from '@docusaurus/router';
import './NavbarAuth.css';

/**
 * Navbar authentication component
 * Shows user info and logout when authenticated
 * Shows sign in/up links when not authenticated
 */
export default function NavbarAuth(): JSX.Element {
  return (
    <BrowserOnly fallback={<div />}>
      {() => <NavbarAuthContent />}
    </BrowserOnly>
  );
}

function NavbarAuthContent(): JSX.Element {
  const { user, isAuthenticated, logout } = useAuthContext();
  const history = useHistory();

  const handleLogout = async () => {
    await logout();
    history.push('/');
  };

  const handleSignIn = () => {
    history.push('/signin');
  };

  const handleSignUp = () => {
    history.push('/signup');
  };

  if (isAuthenticated && user) {
    return (
      <div className="navbar-auth">
        <button
          onClick={() => history.push('/profile')}
          className="navbar-auth__username"
        >
          {user.name || user.email}
        </button>
        <button
          onClick={handleLogout}
          className="navbar-auth__logout"
        >
          Logout
        </button>
      </div>
    );
  }

  return (
    <div className="navbar-auth">
      <button
        onClick={handleSignIn}
        className="navbar-auth__signin"
      >
        Sign In
      </button>
      <button
        onClick={handleSignUp}
        className="navbar-auth__signup"
      >
        Sign Up
      </button>
    </div>
  );
}
