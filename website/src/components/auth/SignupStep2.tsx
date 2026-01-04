import React from 'react';
import { Controller, Control, FieldErrors } from 'react-hook-form';
import { useTranslation } from 'react-i18next';
import styles from '../../pages/auth.module.css';

interface SignupStep2Props {
  control: Control<any>;
  errors: FieldErrors<any>;
}

const PROGRAMMING_LANGUAGES = [
  { label: 'Python', value: 'python' },
  { label: 'JavaScript', value: 'javascript' },
  { label: 'TypeScript', value: 'typescript' },
  { label: 'C++', value: 'cpp' },
  { label: 'Java', value: 'java' },
  { label: 'C#', value: 'csharp' },
  { label: 'Go', value: 'go' },
  { label: 'Rust', value: 'rust' },
];

const AI_ML_LEVELS = [
  { label: 'None', value: 'none' },
  { label: 'Beginner', value: 'beginner' },
  { label: 'Intermediate', value: 'intermediate' },
  { label: 'Advanced', value: 'advanced' },
];

const PROGRAMMING_YEARS_OPTIONS = [
  { label: 'No experience', value: '0' },
  { label: '1-3 years', value: '1-3' },
  { label: '3-5 years', value: '3-5' },
  { label: '5-10 years', value: '5-10' },
  { label: '10+ years', value: '10+' },
];

/**
 * Signup Step 2: Software Background
 * - Programming years
 * - Programming languages
 * - AI/ML experience level
 */
export const SignupStep2: React.FC<SignupStep2Props> = ({ control, errors }) => {
  useTranslation(['forms', 'auth']);

  return (
    <div className={styles.step}>
      <h2 className={styles.stepTitle}>Software Background</h2>

      {/* Programming Years */}
      <div className={styles.formGroup}>
        <label htmlFor="programmingYears" className={styles.label}>
          Programming Experience <span className={styles.required}>*</span>
        </label>
        <p className={styles.helpText}>How many years of programming experience do you have?</p>
        <Controller
          name="programmingYears"
          control={control}
          rules={{ required: 'Field is required' }}
          render={({ field }) => (
            <>
              <select
                {...field}
                id="programmingYears"
                className={`${styles.select} ${errors.programmingYears ? styles.error : ''}`}
                aria-label="Programming Experience"
              >
                <option value="">
                  Select your experience level
                </option>
                {PROGRAMMING_YEARS_OPTIONS.map(({ label, value }) => (
                  <option key={value} value={value}>{label}</option>
                ))}
              </select>
              {errors.programmingYears && (
                <span className={styles.errorText}>{errors.programmingYears.message}</span>
              )}
            </>
          )}
        />
      </div>

      {/* Programming Languages */}
      <div className={styles.formGroup}>
        <label className={styles.label}>
          Programming Languages <span className={styles.required}>*</span>
        </label>
        <p className={styles.helpText}>Which languages are you proficient in?</p>
        <Controller
          name="languages"
          control={control}
          rules={{
            required: 'Field is required',
            validate: (value) => value && value.length > 0 || 'Field is required',
          }}
          render={({ field: { value, onChange } }) => (
            <>
              <div className={styles.checkboxGroup}>
                {PROGRAMMING_LANGUAGES.map(({ label, value: langValue }) => (
                  <label key={langValue} className={styles.checkbox}>
                    <input
                      type="checkbox"
                      checked={value?.includes(langValue) || false}
                      onChange={(e) => {
                        if (e.target.checked) {
                          onChange([...(value || []), langValue]);
                        } else {
                          onChange(value?.filter((v: string) => v !== langValue) || []);
                        }
                      }}
                      aria-label={label}
                    />
                    <span>{label}</span>
                  </label>
                ))}
              </div>
              {errors.languages && (
                <span className={styles.errorText}>{errors.languages.message}</span>
              )}
            </>
          )}
        />
      </div>

      {/* AI/ML Level */}
      <div className={styles.formGroup}>
        <label className={styles.label}>
          AI/ML Experience <span className={styles.required}>*</span>
        </label>
        <Controller
          name="aiMlLevel"
          control={control}
          rules={{ required: 'Field is required' }}
          render={({ field }) => (
            <>
              <div className={styles.radioGroup}>
                {AI_ML_LEVELS.map(({ label, value: levelValue }) => (
                  <label key={levelValue} className={styles.radio}>
                    <input
                      type="radio"
                      {...field}
                      value={levelValue}
                      checked={field.value === levelValue}
                      aria-label={label}
                    />
                    <span>{label}</span>
                  </label>
                ))}
              </div>
              {errors.aiMlLevel && (
                <span className={styles.errorText}>{errors.aiMlLevel.message}</span>
              )}
            </>
          )}
        />
      </div>
    </div>
  );
};

export default SignupStep2;
