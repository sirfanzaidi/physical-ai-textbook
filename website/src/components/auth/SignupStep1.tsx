import React, { useState } from 'react';
import { Controller, Control, FieldErrors } from 'react-hook-form';
import { useTranslation } from 'react-i18next';
import styles from '../../pages/auth.module.css';

interface SignupStep1Props {
  control: Control<any>;
  errors: FieldErrors<any>;
  onEmailCheck?: (email: string) => Promise<boolean>;
}

/**
 * Signup Step 1: Email, Password, Confirm Password, Name
 */
export const SignupStep1: React.FC<SignupStep1Props> = ({ control, errors, onEmailCheck }) => {
  useTranslation(['auth', 'errors']);
  const [emailCheckError, setEmailCheckError] = useState<string | null>(null);
  const [passwordStrength, setPasswordStrength] = useState<'weak' | 'fair' | 'good' | 'strong' | null>(null);

  const calculatePasswordStrength = (password: string) => {
    if (!password) {
      setPasswordStrength(null);
      return;
    }

    let strength = 0;
    if (password.length >= 12) strength++;
    if (password.length >= 16) strength++;
    if (/[a-z]/.test(password) && /[A-Z]/.test(password)) strength++;
    if (/\d/.test(password)) strength++;
    if (/[!@#$%^&*]/.test(password)) strength++;

    if (strength <= 1) setPasswordStrength('weak');
    else if (strength <= 2) setPasswordStrength('fair');
    else if (strength <= 3) setPasswordStrength('good');
    else setPasswordStrength('strong');
  };

  const handleEmailBlur = async (email: string) => {
    if (onEmailCheck && email) {
      const exists = await onEmailCheck(email);
      if (exists) {
        setEmailCheckError('Email already exists');
      } else {
        setEmailCheckError(null);
      }
    }
  };

  return (
    <div className={styles.step}>
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
                className={`${styles.input} ${errors.email || emailCheckError ? styles.error : ''}`}
                aria-label="Email"
                onBlur={(e) => {
                  field.onBlur();
                  handleEmailBlur(e.target.value);
                }}
              />
              {emailCheckError && <span className={styles.errorText}>{emailCheckError}</span>}
              {errors.email && <span className={styles.errorText}>{errors.email.message}</span>}
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
              value: 12,
              message: 'Password must be at least 12 characters',
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
                onChange={(e) => {
                  field.onChange(e);
                  calculatePasswordStrength(e.target.value);
                }}
              />
              {passwordStrength && (
                <div className={`${styles.strengthMeter} ${styles[`strength-${passwordStrength}`]}`}>
                  <span>{passwordStrength.charAt(0).toUpperCase() + passwordStrength.slice(1)}</span>
                </div>
              )}
              {errors.password && <span className={styles.errorText}>{errors.password.message}</span>}
            </>
          )}
        />
      </div>

      {/* Confirm Password Field */}
      <div className={styles.formGroup}>
        <label htmlFor="confirmPassword" className={styles.label}>
          Confirm Password <span className={styles.required}>*</span>
        </label>
        <Controller
          name="confirmPassword"
          control={control}
          rules={{
            required: 'Confirm password is required',
          }}
          render={({ field }) => (
            <>
              <input
                {...field}
                id="confirmPassword"
                type="password"
                placeholder="Confirm your password"
                className={`${styles.input} ${errors.confirmPassword ? styles.error : ''}`}
                aria-label="Confirm Password"
              />
              {errors.confirmPassword && (
                <span className={styles.errorText}>{errors.confirmPassword.message}</span>
              )}
            </>
          )}
        />
      </div>

      {/* Name Field */}
      <div className={styles.formGroup}>
        <label htmlFor="name" className={styles.label}>
          Full Name <span className={styles.required}>*</span>
        </label>
        <Controller
          name="name"
          control={control}
          rules={{
            required: 'Name is required',
            minLength: {
              value: 2,
              message: 'Name must be at least 2 characters',
            },
          }}
          render={({ field }) => (
            <>
              <input
                {...field}
                id="name"
                type="text"
                placeholder="Enter your full name"
                className={`${styles.input} ${errors.name ? styles.error : ''}`}
                aria-label="Full Name"
              />
              {errors.name && <span className={styles.errorText}>{errors.name.message}</span>}
            </>
          )}
        />
      </div>
    </div>
  );
};

export default SignupStep1;
