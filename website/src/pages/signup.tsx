import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useForm, Controller } from 'react-hook-form';
import { useAuthContext } from '../context/AuthContext';
import styles from './auth.module.css';
import { API_BASE_URL } from '../lib/apiConfig';

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

const PROGRAMMING_OPTIONS = [
  'Python',
  'JavaScript',
  'TypeScript',
  'C++',
  'Java',
  'C#',
  'Go',
  'Rust',
];

const FRAMEWORKS_OPTIONS = [
  'React',
  'Vue',
  'Angular',
  'Django',
  'FastAPI',
  'Next.js',
  'Express',
  'Spring Boot',
];

const HARDWARE_OPTIONS = [
  'Arduino',
  'Raspberry Pi',
  'NVIDIA Jetson',
  'Microcontrollers',
  'Sensors & Actuators',
  'Robot Kits',
  'No Experience',
];

export default function SignupPage(): JSX.Element {
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

  const [submitted, setSubmitted] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const password = watch('password');

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
        } catch {
          setError('Signup failed. Please try again.');
        }
        return;
      }

      const responseData = await response.json();
      // Store the token
      if (responseData.token) {
        localStorage.setItem('auth_token', responseData.token);
        // Reload the page to refresh auth context
        setTimeout(() => {
          window.location.href = '/';
        }, 500);
      }

      setSubmitted(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setLoading(false);
    }
  };

  if (submitted) {
    return (
      <Layout title="Signup Success" description="Account created successfully">
        <div className={styles.authContainer}>
          <div className={styles.successMessage}>
            <h1>Welcome!</h1>
            <p>Your account has been created successfully.</p>
            <p>Redirecting to your dashboard...</p>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Sign Up" description="Create an account and personalize your learning">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1>Create Your Account</h1>
          <p className={styles.subtitle}>Tell us about yourself to personalize your learning experience</p>

          {error && <div className={styles.errorMessage}>{error}</div>}

          <form onSubmit={handleSubmit(onSubmit)}>
            {/* Basic Info */}
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

            {/* Learning Background */}
            <div className={styles.formSection}>
              <h2>Your Background</h2>

              <div className={styles.formGroup}>
                <label>Experience Level *</label>
                <Controller
                  name="experience_level"
                  control={control}
                  render={({ field }) => (
                    <select {...field}>
                      <option value="beginner">Beginner - New to programming/robotics</option>
                      <option value="intermediate">Intermediate - Some experience</option>
                      <option value="advanced">Advanced - Extensive experience</option>
                    </select>
                  )}
                />
              </div>

              <div className={styles.formGroup}>
                <label>Programming Languages (Select all that apply)</label>
                <Controller
                  name="programmingBackgrounds"
                  control={control}
                  render={({ field: { value, onChange } }) => (
                    <div className={styles.checkboxGroup}>
                      {PROGRAMMING_OPTIONS.map((option) => (
                        <label key={option} className={styles.checkbox}>
                          <input
                            type="checkbox"
                            checked={value.includes(option)}
                            onChange={(e) => {
                              if (e.target.checked) {
                                onChange([...value, option]);
                              } else {
                                onChange(value.filter((v) => v !== option));
                              }
                            }}
                          />
                          {option}
                        </label>
                      ))}
                    </div>
                  )}
                />
              </div>

              <div className={styles.formGroup}>
                <label>Frameworks & Libraries (Select all that apply)</label>
                <Controller
                  name="frameworksKnown"
                  control={control}
                  render={({ field: { value, onChange } }) => (
                    <div className={styles.checkboxGroup}>
                      {FRAMEWORKS_OPTIONS.map((option) => (
                        <label key={option} className={styles.checkbox}>
                          <input
                            type="checkbox"
                            checked={value.includes(option)}
                            onChange={(e) => {
                              if (e.target.checked) {
                                onChange([...value, option]);
                              } else {
                                onChange(value.filter((v) => v !== option));
                              }
                            }}
                          />
                          {option}
                        </label>
                      ))}
                    </div>
                  )}
                />
              </div>
            </div>

            {/* Hardware & Robotics Background */}
            <div className={styles.formSection}>
              <h2>Hardware & Robotics Experience</h2>

              <div className={styles.formGroup}>
                <label>Hardware Experience (Select all that apply)</label>
                <Controller
                  name="hardwareExperience"
                  control={control}
                  render={({ field: { value, onChange } }) => (
                    <div className={styles.checkboxGroup}>
                      {HARDWARE_OPTIONS.map((option) => (
                        <label key={option} className={styles.checkbox}>
                          <input
                            type="checkbox"
                            checked={value.includes(option)}
                            onChange={(e) => {
                              if (e.target.checked) {
                                onChange([...value, option]);
                              } else {
                                onChange(value.filter((v) => v !== option));
                              }
                            }}
                          />
                          {option}
                        </label>
                      ))}
                    </div>
                  )}
                />
              </div>

              <div className={styles.formGroup}>
                <label>What interests you most about robotics?</label>
                <Controller
                  name="roboticsInterest"
                  control={control}
                  render={({ field }) => (
                    <textarea
                      {...field}
                      placeholder="e.g., Humanoid robotics, AI, mechanical design, control systems..."
                      rows={3}
                    />
                  )}
                />
              </div>
            </div>

            <button type="submit" className={styles.submitButton} disabled={loading}>
              {loading ? 'Creating Account...' : 'Create Account'}
            </button>

            <p className={styles.signinLink}>
              Already have an account? <a href="/signin">Sign In</a>
            </p>
          </form>
        </div>
      </div>
    </Layout>
  );
}
