import React from 'react';
import NavbarDefault from '@theme-original/Navbar';
import NavbarAuth from '@site/src/components/NavbarAuth';
import { useAuthContext } from '@site/src/context/AuthContext';
import BrowserOnly from '@docusaurus/BrowserOnly';

/**
 * Custom Navbar wrapper with authentication
 */
export default function Navbar(props: any): JSX.Element {
  return (
    <BrowserOnly fallback={<NavbarDefault {...props} />}>
      {() => <NavbarWithAuth {...props} />}
    </BrowserOnly>
  );
}

function NavbarWithAuth(props: any): JSX.Element {
  const { isAuthenticated } = useAuthContext();

  // Hide default Sign In/Sign Up links when authenticated
  React.useEffect(() => {
    if (isAuthenticated) {
      document.body.classList.add('user-authenticated');
    } else {
      document.body.classList.remove('user-authenticated');
    }
  }, [isAuthenticated]);

  return (
    <div className="custom-navbar-wrapper">
      <NavbarDefault {...props} />
      <div className="custom-navbar-auth-container">
        <NavbarAuth />
      </div>
    </div>
  );
}