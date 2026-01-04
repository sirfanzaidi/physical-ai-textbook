import React, { useState } from 'react';
import { useForm } from 'react-hook-form';
import { useTranslation } from 'react-i18next';
import { useHistory } from 'react-router-dom';
import { signUp } from '../../lib/authClient';
import SignupStep1 from './SignupStep1';
import SignupStep2 from './SignupStep2';
import SignupStep3 from './SignupStep3';
import SignupStep4 from './SignupStep4';
import styles from '../../pages/auth.module.css';

interface MultiStepSignupFormProps {
  onSuccess?: (userId: string) => void;
  redirectUrl?: string;
  apiBaseUrl?: string;
}

interface SignupFormData {
  email: string;
  password: string;
  confirmPassword: string;
  name: string;
  programming_backgrounds: string[];
  frameworks_known: string[];
  hardware_experience: string[];
  robotics_interest: string;
  experience_level: string;
  language_preference: string;
}

/**
 * MultiStepSignupForm Component
 * Manages 4-step signup flow with form validation and submission
 */
export const MultiStepSignupForm: React.FC<MultiStepSignupFormProps> = ({
  onSuccess,
  redirectUrl = '/profile',
  apiBaseUrl = 'http://localhost:8000',
}) => {
  const { i18n } = useTranslation(['auth', 'forms', 'errors']);
  const history = useHistory();
  const [currentStep, setCurrentStep] = useState(1);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [globalError, setGlobalError] = useState<string | null>(null);

  const {
    control,
    handleSubmit,
    trigger,
    watch,
    formState: { errors, isValid },
  } = useForm<SignupFormData>({
    mode: 'onBlur',
    defaultValues: {
      email: '',
      password: '',
      confirmPassword: '',
      name: '',
      programming_backgrounds: [],
      frameworks_known: [],
      hardware_experience: [],
      robotics_interest: '',
      experience_level: 'beginner',
      language_preference: 'en', // Assuming default English
    },
  });

  const passwordValue = watch('password');
  const confirmPasswordValue = watch('confirmPassword');

  // Validate step-specific fields
  const validateStep = async (step: number): Promise<boolean> => {
    switch (step) {
      case 1:
        return await trigger(['email', 'password', 'confirmPassword', 'name']);
      case 2:
        return await trigger(['programming_backgrounds', 'frameworks_known']);
      case 3:
        return await trigger(['hardware_experience', 'robotics_interest']);
      case 4:
        return await trigger(['experience_level', 'language_preference']);
      default:
        return true;
    }
  };

  // Validate password match
  const isPasswordMatch = passwordValue === confirmPasswordValue;

  const handleNext = async () => {
    if (currentStep === 1 && !isPasswordMatch) {
      setGlobalError('Passwords do not match');
      return;
    }

    const isStepValid = await validateStep(currentStep);
    if (isStepValid && currentStep < 4) {
      setCurrentStep(currentStep + 1);
      setGlobalError(null);
      // Scroll to top
      window.scrollTo({ top: 0, behavior: 'smooth' });
    }
  };

  const handleBack = () => {
    if (currentStep > 1) {
      setCurrentStep(currentStep - 1);
      setGlobalError(null);
      window.scrollTo({ top: 0, behavior: 'smooth' });
    }
  };

  const onSubmit = async (data: SignupFormData) => {
    if (!isPasswordMatch) {
      setGlobalError('Passwords do not match');
      return;
    }

    try {
      setIsSubmitting(true);
      setGlobalError(null);

      // Prepare payload
      const payload = {
        email: data.email,
        password: data.password,
        name: data.name,
        language_preference: i18n.language,
        programming_backgrounds: data.programming_backgrounds,
        frameworks_known: data.frameworks_known,
        hardware_experience: data.hardware_experience,
        robotics_interest: data.robotics_interest,
        experience_level: data.experience_level,
      };

      // Call signup API
      const response = await signUp.email({
        email: payload.email,
        password: payload.password,
        name: payload.name,
        data: {
          language_preference: payload.language_preference,
          programming_backgrounds: payload.programming_backgrounds,
          frameworks_known: payload.frameworks_known,
          hardware_experience: payload.hardware_experience,
          robotics_interest: payload.robotics_interest,
          experience_level: payload.experience_level,
        },
      });

      if (response.error) {
        setGlobalError(response.error.message || 'Signup failed. Please try again.');
        setIsSubmitting(false);
        return;
      }

      // Success
      if (response.data?.user?.id) {
        if (onSuccess) {
          onSuccess(response.data.user.id);
        }
        // Redirect to profile or specified URL
        history.push(redirectUrl);
      }
    } catch (error) {
      console.error('Signup error:', error);
      setGlobalError(
        error instanceof Error
          ? error.message
          : 'An unexpected error occurred. Please try again.'
      );
      setIsSubmitting(false);
    }
  };

  const progressPercentage = (currentStep / 4) * 100;

  return (
    <div className={styles.formContainer}>
      {/* Header */}
      <div className={styles.header}>
        <h1 className={styles.title}>Sign Up</h1>
        <p className={styles.subtitle}>Create your account in 4 steps</p>
      </div>

      {/* Progress Indicator */}
      <div className={styles.progressContainer}>
        <div className={styles.progressBar}>
          <div
            className={styles.progressFill}
            style={{ width: `${progressPercentage}%` }}
            role="progressbar"
            aria-valuenow={currentStep}
            aria-valuemin={1}
            aria-valuemax={4}
          />
        </div>
        <div className={styles.stepIndicator}>
          Step {currentStep} of 4
        </div>
      </div>

      {/* Global Error Message */}
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
        {/* Step Content */}
        {currentStep === 1 && <SignupStep1 control={control} errors={errors} />}
        {currentStep === 2 && <SignupStep2 control={control} errors={errors} />}
        {currentStep === 3 && <SignupStep3 control={control} errors={errors} />}
        {currentStep === 4 && <SignupStep4 control={control} errors={errors} />}

        {/* Navigation Buttons */}
        <div className={styles.buttonGroup}>
          {currentStep > 1 && (
            <button
              type="button"
              onClick={handleBack}
              className={styles.secondaryButton}
              disabled={isSubmitting}
            >
              Back
            </button>
          )}

          <div className={styles.spacer} />

          {currentStep < 4 ? (
            <button
              type="button"
              onClick={handleNext}
              className={styles.primaryButton}
              disabled={isSubmitting}
            >
              Next
            </button>
          ) : (
            <button
              type="submit"
              className={styles.primaryButton}
              disabled={isSubmitting}
            >
              {isSubmitting ? (
                <>
                  <span className={styles.spinner} />
                  Submitting...
                </>
              ) : (
                'Complete Sign Up'
              )}
            </button>
          )}
        </div>
      </form>

      {/* Signin Link */}
      <div className={styles.footer}>
        <p>
          Already have an account?{' '}
          <a href="/signin" className={styles.link}>
            Sign In
          </a>
        </p>
      </div>
    </div>
  );
};

export default MultiStepSignupForm;
