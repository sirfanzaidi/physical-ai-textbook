# Bilingual Authentication - Usage Guide

This guide explains how to use the bilingual authentication components in your application.

---

## 🚀 Quick Start

### 1. Initialize i18n in Your App

In your main `App.tsx` or `index.tsx`:

```typescript
import './i18n/config'; // Initialize i18n on app load
import { BrowserRouter } from 'react-router-dom';

function App() {
  return (
    <BrowserRouter>
      {/* Your app content */}
    </BrowserRouter>
  );
}

export default App;
```

### 2. Create Signup Page

**website/src/pages/signup.tsx**:

```typescript
import React from 'react';
import { MultiStepSignupForm } from '../components/auth/MultiStepSignupForm';
import { LanguageSelector } from '../components/LanguageSelector';
import styles from './auth.module.css';

export default function SignupPage() {
  return (
    <div className={styles.authContainer}>
      <div style={{ position: 'absolute', top: '20px', right: '20px' }}>
        <LanguageSelector />
      </div>
      <div className={styles.authCard}>
        <MultiStepSignupForm
          apiBaseUrl="http://localhost:8000"
          redirectUrl="/dashboard"
        />
      </div>
    </div>
  );
}
```

### 3. Create Signin Page

**website/src/pages/signin.tsx**:

```typescript
import React from 'react';
import { SigninForm } from '../components/auth/SigninForm';
import { LanguageSelector } from '../components/LanguageSelector';
import styles from './auth.module.css';

export default function SigninPage() {
  return (
    <div className={styles.authContainer}>
      <div style={{ position: 'absolute', top: '20px', right: '20px' }}>
        <LanguageSelector />
      </div>
      <div className={styles.authCard}>
        <SigninForm
          apiBaseUrl="http://localhost:8000"
          redirectUrl="/dashboard"
        />
      </div>
    </div>
  );
}
```

---

## 🌍 Using Translations in Components

### Accessing Translations in Components

```typescript
import { useTranslation } from 'react-i18next';

function MyComponent() {
  const { t, i18n } = useTranslation(['auth', 'forms']);

  return (
    <div>
      <h1>{t('auth:signup_title')}</h1>
      <label>{t('forms:email_label')}</label>

      {/* Change language */}
      <button onClick={() => i18n.changeLanguage('en')}>
        English
      </button>
      <button onClick={() => i18n.changeLanguage('ur')}>
        اردو
      </button>
    </div>
  );
}
```

### Translation Namespaces

- **auth**: Authentication-related text (signup, signin, buttons, etc.)
- **forms**: Form labels and questions (background, experience, etc.)
- **chat**: Chat interface text
- **errors**: Error messages
- **common**: General UI elements

### Translation Key Format

```
{namespace}:{key}
```

Examples:
```typescript
t('auth:signup_title')           // "Create Your Account"
t('forms:email_label')           // "Email Address"
t('errors:error_email_exists')  // "This email is already registered"
```

---

## 🎯 Form Components Usage

### MultiStepSignupForm

Complete 4-step signup form:

```typescript
import { MultiStepSignupForm } from '../components/auth/MultiStepSignupForm';

<MultiStepSignupForm
  apiBaseUrl="http://localhost:8000"  // API base URL
  redirectUrl="/dashboard"             // Redirect after success
  onSuccess={(userId) => {
    console.log('Signup successful!', userId);
  }}
/>
```

**Features**:
- ✅ 4-step form flow
- ✅ Step validation
- ✅ Password strength meter
- ✅ Email duplicate checking
- ✅ Bilingual validation messages
- ✅ Progress indicator
- ✅ Back/Next/Submit navigation
- ✅ Loading states

**Data Submitted** (to /api/auth/signup):
```json
{
  "email": "user@example.com",
  "password": "secure_password_12chars",
  "name": "User Name",
  "language_preference": "en",
  "profile": {
    "programming_years": "10+",
    "languages": ["python", "javascript"],
    "ai_ml_level": "intermediate",
    "ros_experience": "basic",
    "hardware_platforms": ["arduino", "raspberry_pi"],
    "humanoid_experience": "simulation",
    "simulation_tools": ["gazebo"],
    "gpu_access": true,
    "hardware_specs": "standard",
    "lab_equipment_access": ["sensors"]
  }
}
```

### SignupStep Components

Individual step components for custom forms:

```typescript
import { SignupStep1 } from '../components/auth/SignupStep1';
import { SignupStep2 } from '../components/auth/SignupStep2';
import { SignupStep3 } from '../components/auth/SignupStep3';
import { SignupStep4 } from '../components/auth/SignupStep4';
import { useForm } from 'react-hook-form';

export function CustomSignupForm() {
  const { control, formState: { errors } } = useForm();

  return (
    <>
      <SignupStep1 control={control} errors={errors} />
      <SignupStep2 control={control} errors={errors} />
      <SignupStep3 control={control} errors={errors} />
      <SignupStep4 control={control} errors={errors} />
    </>
  );
}
```

### SigninForm

Simple email/password signin:

```typescript
import { SigninForm } from '../components/auth/SigninForm';

<SigninForm
  apiBaseUrl="http://localhost:8000"
  redirectUrl="/dashboard"
  onSuccess={(userId) => {
    console.log('Signed in!', userId);
  }}
/>
```

**Features**:
- ✅ Email validation (format check)
- ✅ Password validation (12+ chars)
- ✅ Error handling
- ✅ Loading state
- ✅ Session restoration

---

## 🌐 Language Selector Usage

### Add to Your Navbar

```typescript
import { LanguageSelector } from '../components/LanguageSelector';

function Navbar() {
  return (
    <nav>
      <div>My App</div>
      <LanguageSelector />  {/* Add language toggle */}
    </nav>
  );
}
```

### Manual Language Switching

```typescript
import { useTranslation } from 'react-i18next';

function MyComponent() {
  const { i18n } = useTranslation();

  return (
    <div>
      <button onClick={() => i18n.changeLanguage('en')}>
        Switch to English
      </button>
      <button onClick={() => i18n.changeLanguage('ur')}>
        Switch to Urdu
      </button>

      <p>Current language: {i18n.language}</p>
    </div>
  );
}
```

---

## 🔐 Security Features

### Password Validation

All passwords are validated:
- ✅ Minimum 12 characters required
- ✅ Strength meter shows quality
- ✅ Confirm password must match
- ✅ Bcrypt hashing on backend (12 rounds)

### Email Validation

- ✅ Format validation (RFC 5322)
- ✅ Duplicate email checking
- ✅ Case-insensitive comparison

### CSRF Protection

- ✅ Session-based tokens
- ✅ httpOnly cookies
- ✅ SameSite cookie policy

---

## 🧪 Testing the Components

### Unit Test Example

```typescript
import { render, screen, fireEvent } from '@testing-library/react';
import { MultiStepSignupForm } from './MultiStepSignupForm';

test('renders signup form with English labels', () => {
  render(<MultiStepSignupForm />);

  expect(screen.getByText('Create Your Account')).toBeInTheDocument();
  expect(screen.getByText('Email Address')).toBeInTheDocument();
});

test('switches to Urdu when language changed', async () => {
  const { i18n } = render(<LanguageSelector />);

  await i18n.changeLanguage('ur');

  expect(screen.getByText('اپنا اکاؤنٹ بنائیں')).toBeInTheDocument();
});
```

### E2E Test Example

```typescript
import { test, expect } from '@playwright/test';

test('complete signup flow', async ({ page }) => {
  await page.goto('http://localhost:3000/signup');

  // Fill Step 1
  await page.fill('input[name="email"]', 'test@example.com');
  await page.fill('input[name="password"]', 'SecurePassword123!');
  await page.fill('input[name="confirmPassword"]', 'SecurePassword123!');
  await page.fill('input[name="name"]', 'Test User');
  await page.click('button:has-text("Next")');

  // Fill Step 2
  await page.selectOption('select[name="programmingYears"]', '5-10');
  await page.check('input[value="python"]');
  await page.check('input[value="intermediate"]');
  await page.click('button:has-text("Next")');

  // Continue for steps 3 & 4...
  // Submit
  await page.click('button:has-text("Create Account")');

  // Verify redirect
  await expect(page).toHaveURL('http://localhost:3000/dashboard');
});
```

---

## 🎨 Styling Customization

### Override Default Styles

```css
/* Override form container */
.formContainer {
  max-width: 800px;  /* Wider form */
  border-radius: 16px;  /* More rounded corners */
}

/* Override button colors */
.primaryButton {
  background: linear-gradient(90deg, #667eea 0%, #764ba2 100%);
  /* Change gradient colors here */
}

/* Override input styling */
.input:focus {
  border-color: #667eea;  /* Change focus color */
  box-shadow: 0 0 0 3px rgba(102, 126, 234, 0.1);
}
```

### RTL Customization

```css
/* Custom RTL spacing */
html[dir='rtl'] .formGroup {
  padding-right: 20px;  /* Right padding for RTL */
}

html[dir='rtl'] .errorBanner {
  border-right: 4px solid #f44;  /* Right border for RTL */
  border-left: none;
}
```

---

## 📱 Mobile Optimization

The forms are fully responsive:

### Desktop (>1024px)
```
┌─────────────────────────────┐
│ Language Selector    [EN][UR] │
├─────────────────────────────┤
│ Create Your Account         │
│ Step 1 of 4 ███░░░░░░░░     │
│                             │
│ Email Address *             │
│ [_______________________]   │
│                             │
│ [Back] [____________Next]   │
└─────────────────────────────┘
```

### Mobile (<640px)
```
┌──────────────────┐
│[EN][UR]          │
├──────────────────┤
│ Create Account   │
│ Step 1 of 4      │
│ ███░░░░░░░░     │
│                  │
│ Email Address *  │
│ [_____________]  │
│                  │
│ [Back]           │
│ [Next]           │
└──────────────────┘
```

---

## 🐛 Common Issues & Solutions

### Issue 1: Translations Not Appearing

**Problem**: Form labels showing as `{missing en.forms.email_label}`

**Solution**:
1. Check i18n/config.ts is imported in App.tsx
2. Verify translation keys exist in en.json and ur.json
3. Check namespace is correct: `t('forms:email_label')` not `t('email_label')`

### Issue 2: Language Toggle Not Working

**Problem**: Clicking language buttons doesn't change UI

**Solution**:
1. Ensure LanguageSelector is wrapped in a component that uses `useTranslation()`
2. Check localStorage is not blocked
3. Verify i18n instance is initialized

### Issue 3: RTL Layout Broken

**Problem**: Urdu text appears but layout is wrong

**Solution**:
1. Ensure `document.documentElement.dir = 'rtl'` is set in LanguageSelector
2. Check CSS RTL overrides are applied
3. Test in browser DevTools: `document.documentElement.dir` should be 'rtl'

### Issue 4: Password Strength Meter Not Showing

**Problem**: No strength meter appears below password field

**Solution**:
1. Ensure calculatePasswordStrength() is being called
2. Check strengthMeter CSS classes are loaded
3. Verify translation keys for strength levels exist

---

## 📚 Additional Resources

### Translation Files
- `website/src/i18n/en.json` - English translations
- `website/src/i18n/ur.json` - Urdu translations
- `website/src/i18n/config.ts` - i18n configuration

### Component Files
- `website/src/components/LanguageSelector.tsx`
- `website/src/components/auth/SignupStep1.tsx`
- `website/src/components/auth/SignupStep2.tsx`
- `website/src/components/auth/SignupStep3.tsx`
- `website/src/components/auth/SignupStep4.tsx`
- `website/src/components/auth/MultiStepSignupForm.tsx`
- `website/src/components/auth/SigninForm.tsx`

### Styling
- `website/src/pages/auth.module.css`
- `website/src/components/LanguageSelector.module.css`

### Documentation
- `BILINGUAL_AUTH_IMPLEMENTATION.md` - Complete feature overview
- `BILINGUAL_AUTH_USAGE_GUIDE.md` - This file

---

## ✅ Deployment Checklist

Before deploying to production:

- [ ] i18n initialized in main App component
- [ ] Translation files are complete and validated
- [ ] LanguageSelector added to Navbar
- [ ] Signup and Signin pages created
- [ ] API endpoints working (/api/auth/signup, /api/auth/signin)
- [ ] localStorage is accessible and not blocked
- [ ] RTL styling tested in browser
- [ ] Mobile responsiveness tested
- [ ] Form validation working
- [ ] Error messages displaying correctly
- [ ] Language preference persisting across sessions
- [ ] Both English and Urdu fully tested
- [ ] Accessibility features verified (ARIA labels, keyboard nav)
- [ ] Performance tested (form load time < 2s)
- [ ] Security assessment passed

---

**Last Updated**: 2025-12-18
**Maintained By**: Claude Code
**Language Support**: English (en) + Urdu (اردو)
