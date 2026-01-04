import React from 'react';
import { Controller, Control, FieldErrors } from 'react-hook-form';
import { useTranslation } from 'react-i18next';
import styles from '../../pages/auth.module.css';

interface SignupStep3Props {
  control: Control<any>;
  errors: FieldErrors<any>;
}

const HARDWARE_PLATFORMS = [
  { label: 'Arduino', value: 'arduino' },
  { label: 'Raspberry Pi', value: 'raspberry_pi' },
  { label: 'Jetson', value: 'jetson' },
  { label: 'Microcontrollers', value: 'microcontrollers' },
  { label: 'Sensors', value: 'sensors' },
  { label: 'Robot Kits', value: 'robot_kits' },
];

const SIMULATION_TOOLS = [
  { label: 'Gazebo', value: 'gazebo' },
  { label: 'Isaac Sim', value: 'isaac_sim' },
  { label: 'CoppeliaSim', value: 'coppeliasim' },
];

const ROS_LEVELS = [
  { label: 'None', value: 'none' },
  { label: 'Basic', value: 'basic' },
  { label: 'Intermediate', value: 'intermediate' },
  { label: 'Advanced', value: 'advanced' },
];

const HUMANOID_LEVELS = [
  { label: 'None', value: 'none' },
  { label: 'Simulation Only', value: 'simulation' },
  { label: 'Physical Robot', value: 'physical' },
  { label: 'Research Experience', value: 'research' },
];

/**
 * Signup Step 3: Robotics & Hardware Background
 * - ROS experience
 * - Hardware platforms
 * - Humanoid robot experience
 * - Simulation tools
 */
export const SignupStep3: React.FC<SignupStep3Props> = ({ control, errors }) => {
  useTranslation(['forms', 'auth']);

  return (
    <div className={styles.step}>
      <h2 className={styles.stepTitle}>Robotics & Hardware Background</h2>

      {/* ROS Experience */}
      <div className={styles.formGroup}>
        <label className={styles.label}>
          ROS (Robot Operating System) Experience
        </label>
        <Controller
          name="rosExperience"
          control={control}
          render={({ field }) => (
            <div className={styles.radioGroup}>
              {ROS_LEVELS.map(({ label, value: levelValue }) => (
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
          )}
        />
      </div>

      {/* Hardware Platforms */}
      <div className={styles.formGroup}>
        <label className={styles.label}>
          Hardware Platforms
        </label>
        <p className={styles.helpText}>Which hardware platforms have you worked with?</p>
        <Controller
          name="hardwarePlatforms"
          control={control}
          render={({ field: { value, onChange } }) => (
            <div className={styles.checkboxGroup}>
              {HARDWARE_PLATFORMS.map(({ label, value: hwValue }) => (
                <label key={hwValue} className={styles.checkbox}>
                  <input
                    type="checkbox"
                    checked={value?.includes(hwValue) || false}
                    onChange={(e) => {
                      if (e.target.checked) {
                        onChange([...(value || []), hwValue]);
                      } else {
                        onChange(value?.filter((v: string) => v !== hwValue) || []);
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

      {/* Humanoid Robot Experience */}
      <div className={styles.formGroup}>
        <label className={styles.label}>
          Humanoid Robot Experience
        </label>
        <Controller
          name="humanoidExperience"
          control={control}
          render={({ field }) => (
            <div className={styles.radioGroup}>
              {HUMANOID_LEVELS.map(({ label, value: levelValue }) => (
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
          )}
        />
      </div>

      {/* Simulation Tools */}
      <div className={styles.formGroup}>
        <label className={styles.label}>
          Simulation Tools
        </label>
        <p className={styles.helpText}>Which simulation environments are you familiar with?</p>
        <Controller
          name="simulationTools"
          control={control}
          render={({ field: { value, onChange } }) => (
            <div className={styles.checkboxGroup}>
              {SIMULATION_TOOLS.map(({ label, value: toolValue }) => (
                <label key={toolValue} className={styles.checkbox}>
                  <input
                    type="checkbox"
                    checked={value?.includes(toolValue) || false}
                    onChange={(e) => {
                      if (e.target.checked) {
                        onChange([...(value || []), toolValue]);
                      } else {
                        onChange(value?.filter((v: string) => v !== toolValue) || []);
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
    </div>
  );
};

export default SignupStep3;
