import i18n from 'i18next';
import { initReactI18next } from 'react-i18next';
import enTranslations from './en.json';
import urTranslations from './ur.json';

// Detect browser language (only on client side)
const getBrowserLanguage = (): string => {
  if (typeof navigator === 'undefined') return 'en';
  const browserLang = navigator.language || navigator.languages?.[0] || 'en';
  if (browserLang.startsWith('ur')) {
    return 'ur';
  }
  return 'en';
};

// Initialize i18n
i18n
  .use(initReactI18next)
  .init({
    resources: {
      en: enTranslations,
      ur: urTranslations,
    },
    lng: typeof localStorage !== 'undefined' ? (localStorage.getItem('language_preference') || getBrowserLanguage()) : 'en',
    fallbackLng: 'en',
    defaultNS: 'common',
    ns: ['common', 'auth', 'forms', 'chat', 'errors'],
    interpolation: {
      escapeValue: false, // React already escapes values
    },
    react: {
      useSuspense: false, // Don't use Suspense
    },
  });

export default i18n;
