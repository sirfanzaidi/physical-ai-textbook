import React from 'react';
import { useNavigate } from '@docusaurus/router';
import { useAuthContext } from '@site/src/context/AuthContext';

interface ProtectedRouteProps {
  children: React.ReactNode;
  requiredExperienceLevel?: 'beginner' | 'intermediate' | 'advanced';
}

/**
 * ProtectedRoute component that ensures user is authenticated
 * before rendering children. Redirects to signin if not authenticated.
 */
export const ProtectedRoute: React.FC<ProtectedRouteProps> = ({
  children,
  requiredExperienceLevel,
}) => {
  const { isLoading, isAuthenticated, user } = useAuthContext();
  const navigate = useNavigate();

  if (isLoading) {
    return (
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', minHeight: '100vh' }}>
        <p>Loading...</p>
      </div>
    );
  }

  if (!isAuthenticated) {
    navigate('/signin');
    return null;
  }

  // Check experience level requirement if specified
  if (requiredExperienceLevel && user) {
    const levels = ['beginner', 'intermediate', 'advanced'];
    const userLevelIndex = levels.indexOf(user.experience_level);
    const requiredLevelIndex = levels.indexOf(requiredExperienceLevel);

    if (userLevelIndex < requiredLevelIndex) {
      return (
        <div style={{ padding: '40px', textAlign: 'center' }}>
          <h1>Access Restricted</h1>
          <p>This content requires at least {requiredExperienceLevel} level experience.</p>
          <p>Your current level: {user.experience_level}</p>
        </div>
      );
    }
  }

  return <>{children}</>;
};

export default ProtectedRoute;
