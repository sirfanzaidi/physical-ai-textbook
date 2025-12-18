import React, { useState } from 'react';
import { useForm, Controller } from 'react-hook-form';
import styles from '../../pages/auth.module.css';
import { API_BASE_URL } from '../../lib/apiConfig';
import BackgroundQuestions from './BackgroundQuestions';

interface SignupFormData {
  email: string;
  password: string;
  confirmPassword: string;
  name: string;
  programmingBackgrounds: string[];
  frameworksKnown: string[];
  hardwareExperience: string[];
  roboticsInterest: string;
  experience_level: 'beginner' | 'intermediate' | 'advanced';
}

interface SignUpFormProps {
  onSuccess?: () => void;
  onError?: (error: string) => void;
}

export default function SignUpForm({ onSuccess, onError }: SignUpFormProps): JSX.Element {
  const { control, handleSubmit, watch, formState: { errors } } = useForm<SignupFormData>({
    defaultValues: {
      email: '',
      password: '',
      confirmPassword: '',
      name: '',
      programmingBackgrounds: [],
      frameworksKnown: [],
      hardwareExperience: [],
      roboticsInterest: '',
      experience_level: 'beginner',
    },
  });

  const [step, setStep] = useState<1 | 2>(1);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const password = watch('password');

  const goToStep2 = async () => {
    // Validate step 1 fields
    const step1Valid = await handleSubmit(async (data) => {
      // This validates the form
      setError(null);
      setStep(2);
    })();
  };

  const onSubmit = async (data: SignupFormData) => {
    if (data.password !== data.confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          email: data.email,
          password: data.password,
          name: data.name,
          programming_backgrounds: data.programmingBackgrounds,
          frameworks_known: data.frameworksKnown,
          hardware_experience: data.hardwareExperience,
          robotics_interest: data.roboticsInterest,
          experience_level: data.experience_level,
        }),
      });

      if (!response.ok) {
        try {
          const errorData = await response.json();
          const errorMsg = errorData.detail?.error || errorData.detail || 'Signup failed';
          setError(errorMsg);
          if (onError) onError(errorMsg);
        } catch {
          setError('Signup failed. Please try again.');
          if (onError) onError('Signup failed');
        }
        return;
      }

      const responseData = await response.json();
      // Store the token
      if (responseData.token) {
        localStorage.setItem('auth_token', responseData.token);
        if (onSuccess) {
          onSuccess();
        }
        // Reload the page to refresh auth context
        setTimeout(() => {
          window.location.href = '/';
        }, 500);
      }
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'An error occurred';
      setError(errorMsg);
      if (onError) onError(errorMsg);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.authCard}>
      <h1>Create Your Account</h1>
      <p className={styles.subtitle}>Tell us about yourself to personalize your learning experience</p>

      {error && <div className={styles.errorMessage}>{error}</div>}

      <form onSubmit={handleSubmit(onSubmit)}>
        {/* Step 1: Basic Info */}
        {step === 1 && (
          <>
            <div className={styles.formSection}>
              <h2>Account Details</h2>

              <div className={styles.formGroup}>
                <label>Full Name *</label>
                <Controller
                  name="name"
                  control={control}
                  rules={{ required: 'Name is required' }}
                  render={({ field }) => <input type="text" {...field} placeholder="John Doe" />}
                />
                {errors.name && <span className={styles.error}>{errors.name.message}</span>}
              </div>

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
                  rules={{
                    required: 'Password is required',
                    minLength: { value: 8, message: 'Password must be at least 8 characters' },
                  }}
                  render={({ field }) => <input type="password" {...field} placeholder="••••••••" />}
                />
                {errors.password && <span className={styles.error}>{errors.password.message}</span>}
              </div>

              <div className={styles.formGroup}>
                <label>Confirm Password *</label>
                <Controller
                  name="confirmPassword"
                  control={control}
                  rules={{
                    required: 'Please confirm your password',
                    validate: (value) => value === password || 'Passwords do not match',
                  }}
                  render={({ field }) => <input type="password" {...field} placeholder="••••••••" />}
                />
                {errors.confirmPassword && <span className={styles.error}>{errors.confirmPassword.message}</span>}
              </div>
            </div>

            <button type="button" className={styles.submitButton} onClick={goToStep2}>
              Next: Tell Us About Yourself
            </button>
          </>
        )}

        {/* Step 2: Background Questions */}
        {step === 2 && (
          <>
            <BackgroundQuestions control={control} errors={errors} />

            <div style={{ display: 'flex', gap: '10px', marginTop: '20px' }}>
              <button type="button" className={styles.submitButton} onClick={() => setStep(1)} style={{ flex: 1, opacity: 0.7 }}>
                Back
              </button>
              <button type="submit" className={styles.submitButton} disabled={loading} style={{ flex: 1 }}>
                {loading ? 'Creating Account...' : 'Create Account'}
              </button>
            </div>
          </>
        )}

        {step === 1 && (
          <p className={styles.signinLink}>
            Already have an account? <a href="/signin">Sign In</a>
          </p>
        )}
      </form>
    </div>
  );
}
