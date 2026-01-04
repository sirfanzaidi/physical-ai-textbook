import React from 'react';
import { Controller, Control, FieldErrors } from 'react-hook-form';
import { useTranslation } from 'react-i18next';
import styles from '../../pages/auth.module.css';

interface SignupStep4Props {
  control: Control<any>;
  errors: FieldErrors<any>;
}

const LAB_EQUIPMENT = [
  { label: 'Motion Capture System', value: 'motion_capture' },
  { label: 'Force Feedback Equipment', value: 'force_feedback' },
  { label: 'Advanced Sensors', value: 'sensors' },
];

const HARDWARE_SPECS = [
  { label: 'Basic (2-4 CPU cores, 4-8 GB RAM)', value: 'basic' },
  { label: 'Standard (4-8 CPU cores, 16-32 GB RAM)', value: 'standard' },
  { label: 'High Performance (8+ CPU cores, 32GB+ RAM)', value: 'high_performance' },
];

/**
 * Signup Step 4: Hardware Access & Optional Information
 * - GPU access
 * - Hardware specifications
 * - Lab equipment access
 */
export const SignupStep4: React.FC<SignupStep4Props> = ({ control, errors }) => {
  useTranslation(['forms', 'auth']);

  return (
    <div className={styles.step}>
      <h2 className={styles.stepTitle}>Hardware Access & Setup</h2>
      <p className={styles.stepDescription}>Tell us about your computing setup and lab access</p>

      {/* GPU Access */}
      <div className={styles.formGroup}>
        <Controller
          name="gpuAccess"
          control={control}
          render={({ field }) => (
            <label className={styles.checkbox}>
              <input
                type="checkbox"
                {...field}
                checked={field.value || false}
                aria-label="I have GPU access"
              />
              <span>I have GPU access (NVIDIA CUDA, etc.)</span>
            </label>
          )}
        />
      </div>

      {/* Hardware Specifications */}
      <div className={styles.formGroup}>
        <label htmlFor="hardwareSpecs" className={styles.label}>
          CPU & RAM Configuration
        </label>
        <Controller
          name="hardwareSpecs"
          control={control}
          render={({ field }) => (
            <select
              {...field}
              id="hardwareSpecs"
              className={styles.select}
              aria-label="CPU & RAM Configuration"
            >
              <option value="">
                Select your hardware configuration
              </option>
              {HARDWARE_SPECS.map(({ label, value: specValue }) => (
                <option key={specValue} value={specValue}>
                  {label}
                </option>
              ))}
            </select>
          )}
        />
      </div>

      {/* Lab Equipment Access */}
      <div className={styles.formGroup}>
        <label className={styles.label}>
          Lab Equipment Access
        </label>
        <Controller
          name="labEquipmentAccess"
          control={control}
          render={({ field: { value, onChange } }) => (
            <div className={styles.checkboxGroup}>
              {LAB_EQUIPMENT.map(({ label, value: eqValue }) => (
                <label key={eqValue} className={styles.checkbox}>
                  <input
                    type="checkbox"
                    checked={value?.includes(eqValue) || false}
                    onChange={(e) => {
                      if (e.target.checked) {
                        onChange([...(value || []), eqValue]);
                      } else {
                        onChange(value?.filter((v: string) => v !== eqValue) || []);
                      }
                    }}
                    aria-label={label}
                  />
                  <span>{label}</span>
                </label>
              ))}
            </div>
          )}
        />
      </div>

      {/* Summary Section */}
      <div className={styles.summarySection}>
        <h3 className={styles.summaryTitle}>Profile Summary</h3>
        <p className={styles.summaryText}>
          Please review your information before submitting
        </p>
      </div>
    </div>
  );
};

export default SignupStep4;
