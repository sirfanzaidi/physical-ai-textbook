import React, { useState } from 'react';
import styles from './LanguageSelector.module.css';

/**
 * Language Selector Component
 * Allows users to toggle between English and Urdu
 */
export const LanguageSelector: React.FC = () => {
  const [language, setLanguage] = useState<'en' | 'ur'>(
    (localStorage.getItem('language_preference') as 'en' | 'ur') || 'en'
  );

  const handleLanguageChange = (lang: 'en' | 'ur') => {
    setLanguage(lang);
    localStorage.setItem('language_preference', lang);
    // Set document direction based on language
    document.documentElement.lang = lang;
    document.documentElement.dir = lang === 'ur' ? 'rtl' : 'ltr';
  };

  return (
    <div className={styles.languageSelector}>
      <button
        onClick={() => handleLanguageChange('en')}
        className={`${styles.button} ${language === 'en' ? styles.active : ''}`}
        aria-label="English"
        title="English"
      >
        English
      </button>
      <button
        onClick={() => handleLanguageChange('ur')}
        className={`${styles.button} ${language === 'ur' ? styles.active : ''}`}
        aria-label="اردو"
        title="اردو"
      >
        اردو
      </button>
    </div>
  );
};

export default LanguageSelector;
