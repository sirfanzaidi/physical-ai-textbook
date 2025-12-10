import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useLocation } from '@docusaurus/router';

interface LanguageOption {
  code: string;
  label: string;
}

interface LanguageSwitcherProps {
  className?: string;
}

const LANGUAGES: LanguageOption[] = [
  { code: 'en', label: 'English' },
  { code: 'ur', label: 'اردو' },
];

const STORAGE_KEY = 'preferred_language';

export default function LanguageSwitcher({ className = '' }: LanguageSwitcherProps): JSX.Element {
  const { i18n } = useDocusaurusContext();
  const location = useLocation();
  const [currentLanguage, setCurrentLanguage] = useState<string>('en');
  const [isOpen, setIsOpen] = useState<boolean>(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Detect current language from URL on mount
  useEffect(() => {
    const detectLanguageFromURL = (): string => {
      const pathname = window.location.pathname;

      // Check if pathname starts with /ur/
      if (pathname.startsWith('/ur/') || pathname === '/ur') {
        return 'ur';
      }

      // Check for other locale paths
      for (const locale of i18n.locales) {
        if (pathname.startsWith(`/${locale}/`) || pathname === `/${locale}`) {
          return locale;
        }
      }

      // Default locale (English) - no prefix
      return i18n.defaultLocale;
    };

    const detectedLanguage = detectLanguageFromURL();
    setCurrentLanguage(detectedLanguage);

    // Sync with localStorage
    const storedLanguage = localStorage.getItem(STORAGE_KEY);
    if (storedLanguage !== detectedLanguage) {
      localStorage.setItem(STORAGE_KEY, detectedLanguage);
    }
  }, [i18n.locales, i18n.defaultLocale]);

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    };

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  const handleLanguageChange = (languageCode: string) => {
    if (languageCode === currentLanguage) {
      setIsOpen(false);
      return;
    }

    // Save to localStorage
    localStorage.setItem(STORAGE_KEY, languageCode);

    // Get current pathname without locale prefix
    let currentPath = location.pathname;

    // Remove existing locale prefix if present
    for (const locale of i18n.locales) {
      if (currentPath.startsWith(`/${locale}/`)) {
        currentPath = currentPath.substring(`/${locale}`.length);
        break;
      } else if (currentPath === `/${locale}`) {
        currentPath = '/';
        break;
      }
    }

    // Ensure path starts with /
    if (!currentPath.startsWith('/')) {
      currentPath = '/' + currentPath;
    }

    // Build new URL
    let newPath: string;
    if (languageCode === i18n.defaultLocale) {
      // Default locale (English) - no prefix
      newPath = currentPath;
    } else {
      // Other locales - add prefix
      newPath = `/${languageCode}${currentPath}`;
    }

    // Redirect to new URL
    window.location.href = newPath;
  };

  const getCurrentLanguageLabel = (): string => {
    const language = LANGUAGES.find(lang => lang.code === currentLanguage);
    return language ? language.label : 'English';
  };

  return (
    <div
      className={`language-switcher ${className}`}
      ref={dropdownRef}
      style={{ position: 'relative', display: 'inline-block' }}
    >
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="language-switcher__button"
        aria-label="Select language"
        aria-haspopup="true"
        aria-expanded={isOpen}
        style={{
          padding: '0.5rem 1rem',
          border: '1px solid var(--ifm-color-emphasis-300)',
          borderRadius: '0.375rem',
          backgroundColor: 'var(--ifm-background-color)',
          color: 'var(--ifm-font-color-base)',
          cursor: 'pointer',
          fontSize: '0.875rem',
          fontWeight: 500,
          display: 'flex',
          alignItems: 'center',
          gap: '0.5rem',
          transition: 'all 0.2s ease',
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.borderColor = 'var(--ifm-color-primary)';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.borderColor = 'var(--ifm-color-emphasis-300)';
        }}
      >
        <span aria-hidden="true">🌐</span>
        <span>{getCurrentLanguageLabel()}</span>
        <span
          aria-hidden="true"
          style={{
            transform: isOpen ? 'rotate(180deg)' : 'rotate(0deg)',
            transition: 'transform 0.2s ease',
          }}
        >
          ▼
        </span>
      </button>

      {isOpen && (
        <div
          className="language-switcher__dropdown"
          role="menu"
          style={{
            position: 'absolute',
            top: 'calc(100% + 0.5rem)',
            right: 0,
            backgroundColor: 'var(--ifm-background-color)',
            border: '1px solid var(--ifm-color-emphasis-300)',
            borderRadius: '0.375rem',
            boxShadow: '0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06)',
            minWidth: '150px',
            zIndex: 1000,
            overflow: 'hidden',
          }}
        >
          {LANGUAGES.map((language) => (
            <button
              key={language.code}
              onClick={() => handleLanguageChange(language.code)}
              className="language-switcher__option"
              role="menuitem"
              aria-label={`Switch to ${language.label}`}
              style={{
                width: '100%',
                padding: '0.75rem 1rem',
                border: 'none',
                backgroundColor: currentLanguage === language.code
                  ? 'var(--ifm-color-primary-lightest)'
                  : 'transparent',
                color: currentLanguage === language.code
                  ? 'var(--ifm-color-primary)'
                  : 'var(--ifm-font-color-base)',
                cursor: 'pointer',
                textAlign: 'left',
                fontSize: '0.875rem',
                fontWeight: currentLanguage === language.code ? 600 : 400,
                transition: 'all 0.15s ease',
                display: 'flex',
                alignItems: 'center',
                gap: '0.5rem',
              }}
              onMouseEnter={(e) => {
                if (currentLanguage !== language.code) {
                  e.currentTarget.style.backgroundColor = 'var(--ifm-color-emphasis-100)';
                }
              }}
              onMouseLeave={(e) => {
                if (currentLanguage !== language.code) {
                  e.currentTarget.style.backgroundColor = 'transparent';
                }
              }}
            >
              <span>{language.label}</span>
              {currentLanguage === language.code && (
                <span aria-hidden="true" style={{ marginLeft: 'auto' }}>✓</span>
              )}
            </button>
          ))}
        </div>
      )}
    </div>
  );
}
