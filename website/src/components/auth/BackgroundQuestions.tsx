import React from 'react';
import { Controller, Control, FieldErrors } from 'react-hook-form';
import styles from '../../pages/auth.module.css';

interface BackgroundQuestionsProps {
  control: Control<any>;
  errors: FieldErrors<any>;
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

export default function BackgroundQuestions({ control, errors }: BackgroundQuestionsProps): JSX.Element {
  return (
    <>
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
    </>
  );
}
