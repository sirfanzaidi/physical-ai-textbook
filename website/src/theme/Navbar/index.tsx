import React from 'react';
import NavbarDefault from '@theme-original/Navbar';

/**
 * Custom Navbar wrapper
 * (Auth buttons removed)
 */
export default function Navbar(props: any): JSX.Element {
  return (
    <>
      <NavbarDefault {...props} />
    </>
  );
}