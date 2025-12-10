import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useForm, Controller } from 'react-hook-form';
import { useHistory } from '@docusaurus/router';
import styles from './auth.module.css';
import { API_BASE_URL } from '../lib/apiConfig';

interface SigninFormData {
  email: string;
  password: string;
}

export default function SigninPage(): JSX.Element {
  const { control, handleSubmit, formState: { errors } } = useForm<SigninFormData>({
    defaultValues: {
      email: '',
      password: '',
    },
  });

  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const history = useHistory();

  const onSubmit = async (data: SigninFormData) => {
    setLoading(true);
    setError(null);

    try {
      const apiUrl = typeof window !== 'undefined' && (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1') ? 'http://localhost:8001' : '/api';
      const response = await fetch(`${apiUrl}/auth/signin`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          email: data.email,
          password: data.password,
        }),
      });

      if (!response.ok) {
        try {
          const errorData = await response.json();
          setError(errorData.detail || 'Signin failed');
        } catch {
          setError('Signin failed. Please try again.');
        }
        return;
      }

      // Redirect to home or dashboard on successful signin
      history.push('/');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign In" description="Sign in to your account">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1>Welcome Back</h1>
          <p className={styles.subtitle}>Sign in to your account to continue learning</p>

          {error && <div className={styles.errorMessage}>{error}</div>}

          <form onSubmit={handleSubmit(onSubmit)}>
            <div className={styles.formSection}>
              <div className={styles.formGroup}>
                <label>Email *</label>
                <Controller
                  name="email"
                  control={control}
                  rules={{
                    required: 'Email is required',
                    pattern: { value: /^[A-Z0-9._%+-]+@[A-Z0-9.-]+\.[A-Z]{2,}$/i, message: 'Invalid email' },
                  }}
                  render={({ field }) => <input type="email" {...field} placeholder="you@example.com" />}
                />
                {errors.email && <span className={styles.error}>{errors.email.message}</span>}
              </div>

              <div className={styles.formGroup}>
                <label>Password *</label>
                <Controller
                  name="password"
                  control={control}
                  rules={{ required: 'Password is required' }}
                  render={({ field }) => <input type="password" {...field} placeholder="••••••••" />}
                />
                {errors.password && <span className={styles.error}>{errors.password.message}</span>}
              </div>
            </div>

            <button type="submit" className={styles.submitButton} disabled={loading}>
              {loading ? 'Signing In...' : 'Sign In'}
            </button>

            <p className={styles.signupLink}>
              Don't have an account? <a href="/signup">Sign Up</a>
            </p>
          </form>
        </div>
      </div>
    </Layout>
  );
}
