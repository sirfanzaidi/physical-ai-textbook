import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useNavigate } from '@docusaurus/router';
import { useAuthContext } from '@site/src/context/AuthContext';
import { ProtectedRoute } from '@site/src/components/ProtectedRoute';
import styles from './profile.module.css';

export default function ProfilePage(): JSX.Element {
  return (
    <ProtectedRoute>
      <Layout title="My Profile" description="View and manage your profile">
        <ProfileContent />
      </Layout>
    </ProtectedRoute>
  );
}

function ProfileContent(): JSX.Element {
  const { user, updateProfile } = useAuthContext();
  const navigate = useNavigate();
  const [isEditing, setIsEditing] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);

  const [formData, setFormData] = useState({
    programmingBackgrounds: user?.programmingBackgrounds || [],
    frameworksKnown: user?.frameworksKnown || [],
    hardwareExperience: user?.hardwareExperience || [],
    roboticsInterest: user?.roboticsInterest || '',
    experience_level: user?.experience_level || 'beginner',
  });

  const PROGRAMMING_OPTIONS = [
    'Python', 'JavaScript', 'TypeScript', 'C++', 'Java', 'C#', 'Go', 'Rust',
  ];
  const FRAMEWORKS_OPTIONS = [
    'React', 'Vue', 'Angular', 'Django', 'FastAPI', 'Next.js', 'Express', 'Spring Boot',
  ];
  const HARDWARE_OPTIONS = [
    'Arduino', 'Raspberry Pi', 'NVIDIA Jetson', 'Microcontrollers', 'Sensors & Actuators', 'Robot Kits', 'No Experience',
  ];

  const handleCheckboxChange = (field: 'programmingBackgrounds' | 'frameworksKnown' | 'hardwareExperience', value: string) => {
    setFormData((prev) => {
      const array = prev[field] as string[];
      if (array.includes(value)) {
        return { ...prev, [field]: array.filter((v) => v !== value) };
      } else {
        return { ...prev, [field]: [...array, value] };
      }
    });
  };

  const handleSave = async () => {
    setLoading(true);
    setError(null);
    setSuccess(false);

    try {
      await updateProfile(formData);
      setSuccess(true);
      setIsEditing(false);
      setTimeout(() => setSuccess(false), 3000);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to update profile');
    } finally {
      setLoading(false);
    }
  };

  if (!user) {
    return <div>Loading...</div>;
  }

  return (
    <div className={styles.profileContainer}>
      <div className={styles.profileCard}>
        <div className={styles.header}>
          <h1>My Profile</h1>
          <button
            className={styles.editButton}
            onClick={() => setIsEditing(!isEditing)}
          >
            {isEditing ? 'Cancel' : 'Edit Profile'}
          </button>
        </div>

        {success && (
          <div className={styles.successMessage}>
            ✓ Profile updated successfully!
          </div>
        )}

        {error && (
          <div className={styles.errorMessage}>
            {error}
          </div>
        )}

        {/* User Info */}
        <div className={styles.section}>
          <h2>Account Information</h2>
          <div className={styles.infoGrid}>
            <div className={styles.infoItem}>
              <label>Name</label>
              <p>{user.name}</p>
            </div>
            <div className={styles.infoItem}>
              <label>Email</label>
              <p>{user.email}</p>
            </div>
            <div className={styles.infoItem}>
              <label>Experience Level</label>
              <p className={styles.badge}>{user.experience_level}</p>
            </div>
          </div>
        </div>

        {/* Learning Background */}
        <div className={styles.section}>
          <h2>Learning Background</h2>

          {!isEditing ? (
            <>
              <div className={styles.infoItem}>
                <label>Programming Languages</label>
                <div className={styles.tags}>
                  {formData.programmingBackgrounds.length > 0 ? (
                    formData.programmingBackgrounds.map((lang) => (
                      <span key={lang} className={styles.tag}>{lang}</span>
                    ))
                  ) : (
                    <span className={styles.empty}>Not specified</span>
                  )}
                </div>
              </div>

              <div className={styles.infoItem}>
                <label>Frameworks & Libraries</label>
                <div className={styles.tags}>
                  {formData.frameworksKnown.length > 0 ? (
                    formData.frameworksKnown.map((fw) => (
                      <span key={fw} className={styles.tag}>{fw}</span>
                    ))
                  ) : (
                    <span className={styles.empty}>Not specified</span>
                  )}
                </div>
              </div>
            </>
          ) : (
            <>
              <div className={styles.formGroup}>
                <label>Programming Languages</label>
                <div className={styles.checkboxGrid}>
                  {PROGRAMMING_OPTIONS.map((lang) => (
                    <label key={lang} className={styles.checkbox}>
                      <input
                        type="checkbox"
                        checked={formData.programmingBackgrounds.includes(lang)}
                        onChange={() => handleCheckboxChange('programmingBackgrounds', lang)}
                      />
                      {lang}
                    </label>
                  ))}
                </div>
              </div>

              <div className={styles.formGroup}>
                <label>Frameworks & Libraries</label>
                <div className={styles.checkboxGrid}>
                  {FRAMEWORKS_OPTIONS.map((fw) => (
                    <label key={fw} className={styles.checkbox}>
                      <input
                        type="checkbox"
                        checked={formData.frameworksKnown.includes(fw)}
                        onChange={() => handleCheckboxChange('frameworksKnown', fw)}
                      />
                      {fw}
                    </label>
                  ))}
                </div>
              </div>
            </>
          )}
        </div>

        {/* Hardware & Robotics */}
        <div className={styles.section}>
          <h2>Hardware & Robotics Experience</h2>

          {!isEditing ? (
            <>
              <div className={styles.infoItem}>
                <label>Hardware Experience</label>
                <div className={styles.tags}>
                  {formData.hardwareExperience.length > 0 ? (
                    formData.hardwareExperience.map((hw) => (
                      <span key={hw} className={styles.tag}>{hw}</span>
                    ))
                  ) : (
                    <span className={styles.empty}>Not specified</span>
                  )}
                </div>
              </div>

              <div className={styles.infoItem}>
                <label>Robotics Interest</label>
                <p>{formData.roboticsInterest || 'Not specified'}</p>
              </div>
            </>
          ) : (
            <>
              <div className={styles.formGroup}>
                <label>Experience Level</label>
                <select
                  value={formData.experience_level}
                  onChange={(e) =>
                    setFormData((prev) => ({
                      ...prev,
                      experience_level: e.target.value as any,
                    }))
                  }
                  className={styles.select}
                >
                  <option value="beginner">Beginner</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label>Hardware Experience</label>
                <div className={styles.checkboxGrid}>
                  {HARDWARE_OPTIONS.map((hw) => (
                    <label key={hw} className={styles.checkbox}>
                      <input
                        type="checkbox"
                        checked={formData.hardwareExperience.includes(hw)}
                        onChange={() => handleCheckboxChange('hardwareExperience', hw)}
                      />
                      {hw}
                    </label>
                  ))}
                </div>
              </div>

              <div className={styles.formGroup}>
                <label>Robotics Interest</label>
                <textarea
                  value={formData.roboticsInterest}
                  onChange={(e) =>
                    setFormData((prev) => ({
                      ...prev,
                      roboticsInterest: e.target.value,
                    }))
                  }
                  placeholder="Tell us what interests you most about robotics..."
                  rows={3}
                  className={styles.textarea}
                />
              </div>
            </>
          )}
        </div>

        {isEditing && (
          <div className={styles.actions}>
            <button
              className={styles.saveButton}
              onClick={handleSave}
              disabled={loading}
            >
              {loading ? 'Saving...' : 'Save Changes'}
            </button>
            <button
              className={styles.cancelButton}
              onClick={() => setIsEditing(false)}
              disabled={loading}
            >
              Cancel
            </button>
          </div>
        )}
      </div>
    </div>
  );
}
