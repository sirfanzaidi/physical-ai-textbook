import React, { useState } from 'react';
import { useForm, Controller } from 'react-hook-form';
import { useTranslation } from 'react-i18next';
import { useHistory } from '@docusaurus/router';
import { customSignIn } from '../../lib/customAuthService';
import { useAuthContext } from '../../context/AuthContext';
import styles from '../../pages/auth.module.css';

interface SigninFormData {
  email: string;
  password: string;
}

interface SigninFormProps {
  apiBaseUrl?: string;
  redirectUrl?: string;
  onSuccess?: (userId: string) => void;
}

/**
 * SigninForm Component
 * Simple email/password signin with bilingual support
 */
export const SigninForm: React.FC<SigninFormProps> = ({
  apiBaseUrl = 'http://localhost:8001',
  redirectUrl = '/profile',
  onSuccess,
}) => {
  const { i18n } = useTranslation(['auth', 'errors']);
  const history = useHistory();
  const { login } = useAuthContext();
  const [globalError, setGlobalError] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  const {
    control,
    handleSubmit,
    formState: { errors },
  } = useForm<SigninFormData>({
    mode: 'onBlur',
    defaultValues: {
      email: '',
      password: '',
    },
  });

  const onSubmit = async (data: SigninFormData) => {
    try {
      setIsLoading(true);
      setGlobalError(null);

      const response = await customSignIn({
        email: data.email,
        password: data.password,
      });

      // Update auth context
      login(response.token, response.user as any);

      // Call success callback
      if (onSuccess) {
        onSuccess(response.user.id);
      }

      // Redirect
      setTimeout(() => {
        history.push(redirectUrl);
      }, 500);
    } catch (error) {
      console.error('Signin error:', error);
      let errorMessage = 'Failed to connect to server';
      if (error instanceof Error) {
        errorMessage = error.message;
      }
      setGlobalError(errorMessage);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.formContainer}>
      {/* Header */}
      <div className={styles.header}>
        <h1 className={styles.title}>Sign In</h1>
        <p className={styles.subtitle}>Enter your credentials</p>
      </div>

      {/* Error Message */}
      {globalError && (
        <div className={styles.errorBanner} role="alert">
          <span>{globalError}</span>
          <button
            className={styles.closeBanner}
            onClick={() => setGlobalError(null)}
            aria-label="Close error message"
          >
            ✕
          </button>
        </div>
      )}

      {/* Form */}
      <form onSubmit={handleSubmit(onSubmit)} className={styles.form}>
        {/* Email Field */}
        <div className={styles.formGroup}>
          <label htmlFor="email" className={styles.label}>
            Email <span className={styles.required}>*</span>
          </label>
          <Controller
            name="email"
            control={control}
            rules={{
              required: 'Email is required',
              pattern: {
                value: /^[^\s@]+@[^\s@]+\.[^\s@]+$/,
                message: 'Please enter a valid email',
              },
            }}
            render={({ field }) => (
              <>
                <input
                  {...field}
                  id="email"
                  type="email"
                  placeholder="Enter your email"
                  className={`${styles.input} ${errors.email ? styles.error : ''}`}
                  aria-label="Email"
                  disabled={isLoading}
                  autoComplete="email"
                />
                {errors.email && (
                  <span className={styles.errorText}>{errors.email.message}</span>
                )}
              </>
            )}
          />
        </div>

        {/* Password Field */}
        <div className={styles.formGroup}>
          <label htmlFor="password" className={styles.label}>
            Password <span className={styles.required}>*</span>
          </label>
          <Controller
            name="password"
            control={control}
            rules={{
              required: 'Password is required',
              minLength: {
                value: 4,
                message: 'Password must be at least 4 characters',
              },
            }}
            render={({ field }) => (
              <>
                <input
                  {...field}
                  id="password"
                  type="password"
                  placeholder="Enter your password"
                  className={`${styles.input} ${errors.password ? styles.error : ''}`}
                  aria-label="Password"
                  disabled={isLoading}
                  autoComplete="current-password"
                />
                {errors.password && (
                  <span className={styles.errorText}>{errors.password.message}</span>
                )}
              </>
            )}
          />
        </div>

        {/* Submit Button */}
        <button
          type="submit"
          className={styles.primaryButton}
          disabled={isLoading}
          style={{ width: '100%', marginTop: '20px' }}
        >
          {isLoading ? (
            <>
              <span className={styles.spinner} />
              Submitting...
            </>
          ) : (
            'Sign In'
          )}
        </button>
      </form>

      {/* Links */}
      <div className={styles.footer}>
        <div className={styles.linkGroup}>
          <a href="/forgot-password" className={styles.link}>
            Forgot Password?
          </a>
        </div>

        <div className={styles.signupPrompt}>
          <p>
            Don't have an account?{' '}
            <a href="/signup" className={styles.link}>
              Sign Up
            </a>
          </p>
        </div>
      </div>
    </div>
  );
};

export default SigninForm;
