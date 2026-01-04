---
title: "004 - Bilingual Authentication Extension with Better Auth Integration - Implementation Plan"
feature_id: "004-bilingual-auth-extension"
version: "1.0"
created_date: "2025-12-18"
status: "Draft"
---

# Implementation Plan: Bilingual Authentication Extension with Better Auth

## Executive Summary

This plan outlines a 5-phase implementation to extend the existing authentication system with bilingual (English/Urdu) support, Better Auth integration, and rich user profile collection. The system will enable content personalization, audience gating, and chatbot customization based on user technical background.

**Key Architecture Decisions:**
- Replace custom JWT with Better Auth for enterprise-grade security
- Use Drizzle ORM for type-safe database operations
- Implement i18n for bilingual UI (English/Urdu)
- Multi-step signup form with persistent progress tracking
- Server-side content gating with client-side caching
- User profile context injection into RAG chatbot

**Technology Stack:**
- Backend: FastAPI + Better Auth
- Database: Neon PostgreSQL + Drizzle ORM
- Frontend: React/Docusaurus v3+ with react-i18next
- Auth Framework: Better Auth (replaces custom JWT)
- ORM: Drizzle ORM for type-safe database access

---

## Phase 1: Database Schema & Better Auth Setup

### Objectives
- Extend PostgreSQL schema with user_profiles table
- Set up Better Auth on backend
- Configure Drizzle ORM schema definitions
- Prepare database migrations

### Architecture Decisions

**AD1.1: Better Auth vs. Custom JWT**
- **Decision**: Migrate from custom JWT to Better Auth
- **Rationale**: Better Auth provides bcrypt hashing (12 rounds), session management, email verification, audit logs, and OAuth2 readiness
- **Trade-offs**: Requires learning new API, but gains security compliance and future extensibility
- **Implementation**: Phase 1 sets up Better Auth alongside existing JWT (Phase 5 removes JWT completely)

**AD1.2: Drizzle ORM for Type Safety**
- **Decision**: Use Drizzle ORM for all database operations
- **Rationale**: Provides TypeScript types, migration management, and type-safe queries
- **Alternative Considered**: Raw psycopg2 (current) - loses type safety and requires manual schema tracking
- **Impact**: Better long-term maintainability, clearer data contracts

**AD1.3: Array vs. JSON Columns**
- **Decision**: Use PostgreSQL ARRAY type (not JSON) for `languages`, `hardware_platforms`, `simulation_tools`, `lab_equipment_access`
- **Rationale**: Better query performance, native type support, cleaner Drizzle schema
- **Database**: PostgreSQL arrays with ENUM types for consistency

### Components & Responsibilities

#### Backend: Database Layer (`backend/database/`)

**1. Drizzle Schema Definition** (`backend/database/schema.ts`)

```typescript
import { pgTable, uuid, text, timestamp, boolean, pgEnum } from "drizzle-orm/pg-core";

// Enums
export const programmingYearsEnum = pgEnum("programming_years", ["0", "1-3", "3-5", "5-10", "10+"]);
export const aiMlLevelEnum = pgEnum("ai_ml_level", ["none", "beginner", "intermediate", "advanced"]);
export const rosExperienceEnum = pgEnum("ros_experience", ["none", "basic", "intermediate", "advanced"]);
export const humanoidExperienceEnum = pgEnum("humanoid_experience", ["none", "simulation", "physical", "research"]);
export const hardwareSpecsEnum = pgEnum("hardware_specs", ["basic", "standard", "high-performance"]);
export const languageEnum = pgEnum("language_preference", ["en", "ur"]);

// Users table (extended for Better Auth)
export const users = pgTable("users", {
  id: uuid("id").primaryKey(),
  email: text("email").unique().notNull(),
  name: text("name"),
  emailVerified: boolean("email_verified").default(false),
  createdAt: timestamp("created_at", { mode: "string" }).defaultNow(),
  updatedAt: timestamp("updated_at", { mode: "string" }).defaultNow(),
  // Better Auth fields will be in separate sessions/accounts tables
});

// User Profiles table (new)
export const userProfiles = pgTable("user_profiles", {
  id: uuid("id").primaryKey().defaultRandom(),
  userId: uuid("user_id").notNull().references(() => users.id, { onDelete: "cascade" }),

  // Software Background
  programmingYears: programmingYearsEnum("programming_years"),
  languages: text("languages").array().default([]),
  aiMlLevel: aiMlLevelEnum("ai_ml_level").default("none"),

  // Robotics/Hardware Background
  rosExperience: rosExperienceEnum("ros_experience").default("none"),
  hardwarePlatforms: text("hardware_platforms").array().default([]),
  humanoidExperience: humanoidExperienceEnum("humanoid_experience").default("none"),
  simulationTools: text("simulation_tools").array().default([]),

  // Hardware Access
  gpuAccess: boolean("gpu_access").default(false),
  hardwareSpecs: hardwareSpecsEnum("hardware_specs").default("basic"),
  labEquipmentAccess: text("lab_equipment_access").array().default([]),

  // Preferences
  languagePreference: languageEnum("language_preference").default("en"),

  // Metadata
  createdAt: timestamp("created_at", { mode: "string" }).defaultNow(),
  updatedAt: timestamp("updated_at", { mode: "string" }).defaultNow(),
});

// Better Auth Sessions (managed by Better Auth library)
export const sessions = pgTable("sessions", {
  id: text("id").primaryKey(),
  userId: uuid("user_id").notNull().references(() => users.id, { onDelete: "cascade" }),
  expiresAt: timestamp("expires_at").notNull(),
  createdAt: timestamp("created_at").defaultNow(),
});

// Audit Logs for compliance
export const auditLogs = pgTable("audit_logs", {
  id: uuid("id").primaryKey().defaultRandom(),
  userId: uuid("user_id").references(() => users.id, { onDelete: "set null" }),
  action: text("action").notNull(), // 'signup', 'signin', 'profile_update', 'content_access'
  resource: text("resource").notNull(), // 'user', 'profile', 'content'
  details: text("details"), // JSON string with context
  ipAddress: text("ip_address"),
  userAgent: text("user_agent"),
  createdAt: timestamp("created_at").defaultNow(),
});
```

**2. Database Migration** (`backend/database/migrations/002_bilingual_auth_schema.sql`)

```sql
-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- Create enums
CREATE TYPE programming_years AS ENUM ('0', '1-3', '3-5', '5-10', '10+');
CREATE TYPE ai_ml_level AS ENUM ('none', 'beginner', 'intermediate', 'advanced');
CREATE TYPE ros_experience AS ENUM ('none', 'basic', 'intermediate', 'advanced');
CREATE TYPE humanoid_experience AS ENUM ('none', 'simulation', 'physical', 'research');
CREATE TYPE hardware_specs AS ENUM ('basic', 'standard', 'high-performance');
CREATE TYPE language_preference AS ENUM ('en', 'ur');

-- Create user_profiles table
CREATE TABLE user_profiles (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,

  programming_years programming_years,
  languages TEXT[] DEFAULT '{}',
  ai_ml_level ai_ml_level DEFAULT 'none',

  ros_experience ros_experience DEFAULT 'none',
  hardware_platforms TEXT[] DEFAULT '{}',
  humanoid_experience humanoid_experience DEFAULT 'none',
  simulation_tools TEXT[] DEFAULT '{}',

  gpu_access BOOLEAN DEFAULT FALSE,
  hardware_specs hardware_specs DEFAULT 'basic',
  lab_equipment_access TEXT[] DEFAULT '{}',

  language_preference language_preference DEFAULT 'en',

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create sessions table for Better Auth
CREATE TABLE sessions (
  id TEXT PRIMARY KEY,
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  expires_at TIMESTAMP NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create audit_logs table
CREATE TABLE audit_logs (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE SET NULL,
  action TEXT NOT NULL,
  resource TEXT NOT NULL,
  details TEXT,
  ip_address TEXT,
  user_agent TEXT,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for performance
CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
CREATE INDEX idx_user_profiles_ai_ml_level ON user_profiles(ai_ml_level);
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);
CREATE INDEX idx_audit_logs_user_id ON audit_logs(user_id);
CREATE INDEX idx_audit_logs_action ON audit_logs(action);
CREATE INDEX idx_audit_logs_created_at ON audit_logs(created_at);
```

#### Backend: Better Auth Configuration (`backend/auth/better_auth_config.ts`)

```typescript
import { betterAuth } from "better-auth";
import { postgresAdapter } from "better-auth/adapters/postgres";

export const auth = betterAuth({
  database: postgresAdapter(process.env.DATABASE_URL),

  secret: process.env.BETTER_AUTH_SECRET,

  emailAndPassword: {
    enabled: true,
    requireEmailVerification: process.env.ENABLE_EMAIL_VERIFICATION === "true",
    minPasswordLength: 12,
  },

  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60, // 1 hour
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60 * 1000, // 5 minutes
    },
  },

  baseURL: process.env.BACKEND_URL,
  trustedOrigins: [process.env.FRONTEND_URL],

  plugins: [
    // Audit logging plugin
    customAuditPlugin(),
  ],
});

// Custom audit logging plugin
function customAuditPlugin() {
  return {
    id: "audit",
    hooks: {
      after: [
        {
          matcher: (ctx) => ctx.path.includes("/auth/"),
          handler: async (ctx) => {
            // Log auth events to audit_logs table
            const action = ctx.path.split("/").pop();
            await logAuditEvent(
              ctx.request.user?.id,
              action,
              "auth",
              ctx.request.headers.get("user-agent"),
              ctx.request.ip
            );
          },
        },
      ],
    },
  };
}
```

#### Backend: User Service with Drizzle (`backend/services/user_service.py`)

Update to use Drizzle ORM operations:

```typescript
import { db } from "../database";
import { users, userProfiles, auditLogs } from "../database/schema";
import { eq } from "drizzle-orm";

export class UserService {
  async createUserProfile(
    userId: string,
    profileData: {
      programmingYears: string;
      languages: string[];
      aiMlLevel: string;
      rosExperience: string;
      hardwarePlatforms: string[];
      humanoidExperience: string;
      simulationTools: string[];
      gpuAccess: boolean;
      hardwareSpecs: string;
      labEquipmentAccess: string[];
      languagePreference: string;
    }
  ) {
    const profile = await db
      .insert(userProfiles)
      .values({
        userId,
        ...profileData,
      })
      .returning();

    await this.logAuditEvent(userId, "profile_creation", "profile", {
      profile_data: profileData,
    });

    return profile[0];
  }

  async getUserProfile(userId: string) {
    return await db.query.userProfiles.findFirst({
      where: eq(userProfiles.userId, userId),
    });
  }

  async updateUserProfile(userId: string, profileData: Partial<typeof profileData>) {
    const updated = await db
      .update(userProfiles)
      .set({
        ...profileData,
        updatedAt: new Date().toISOString(),
      })
      .where(eq(userProfiles.userId, userId))
      .returning();

    await this.logAuditEvent(userId, "profile_update", "profile", {
      updated_fields: Object.keys(profileData),
    });

    return updated[0];
  }

  async logAuditEvent(
    userId: string | null,
    action: string,
    resource: string,
    details: Record<string, any>,
    ipAddress?: string,
    userAgent?: string
  ) {
    await db.insert(auditLogs).values({
      userId,
      action,
      resource,
      details: JSON.stringify(details),
      ipAddress,
      userAgent,
    });
  }
}
```

### Deliverables (Phase 1)
- ✓ Drizzle ORM schema definition (`schema.ts`)
- ✓ Database migration SQL file (`002_bilingual_auth_schema.sql`)
- ✓ Better Auth configuration (`better_auth_config.ts`)
- ✓ Updated UserService with Drizzle operations
- ✓ Audit logging infrastructure in place
- ✓ All migrations tested in development environment

### Success Criteria
- Database migration runs without errors
- user_profiles table with all required columns and indexes created
- Better Auth configured and sessions table initialized
- Audit logs captured for all authentication events
- Drizzle schema compiles with TypeScript type checking

---

## Phase 2: Bilingual UI Infrastructure & Frontend Components

### Objectives
- Set up i18n framework for English/Urdu translations
- Create reusable multi-step form wizard component
- Build 4-step signup flow UI with bilingual support
- Implement language preference persistence
- Create translation files for all form labels and messages

### Architecture Decisions

**AD2.1: i18n Framework Selection**
- **Decision**: Use `react-i18next` with `i18next` backend loader
- **Rationale**: Industry standard, good Docusaurus integration, supports lazy loading translations
- **Alternative**: Zustand store with manual translations - rejected due to maintenance burden
- **Implementation**: Namespace-based organization (common, auth, forms, errors)

**AD2.2: Form Component Structure**
- **Decision**: Separate reusable components: `MultiStepForm` (controller), `SignupStep1` (credentials), `SignupStep2` (software bg), `SignupStep3` (robotics bg), `SignupStep4` (hardware access)
- **Rationale**: Easier testing, reusability for profile editing, clear separation of concerns
- **State Management**: react-hook-form for form state, Context for multi-step state

**AD2.3: Language Preference Storage**
- **Decision**: localStorage (client) + user_profiles.language_preference (server)
- **Rationale**: Fast client-side switching (no server round-trip), synced with profile on signup
- **Sync Mechanism**: On profile creation, write language_preference to DB; on profile load, update localStorage if different

### Components & Responsibilities

#### Frontend: i18n Configuration (`website/src/lib/i18n/config.ts`)

```typescript
import i18n from "i18next";
import { initReactI18next } from "react-i18next";
import HttpBackend from "i18next-http-backend";

i18n
  .use(HttpBackend)
  .use(initReactI18next)
  .init({
    fallbackLng: "en",
    defaultNS: "common",
    ns: ["common", "auth", "forms", "errors", "chat"],
    backend: {
      loadPath: "/locales/{{lng}}/{{ns}}.json",
    },
    interpolation: {
      escapeValue: false,
    },
    react: {
      useSuspense: false,
    },
  });

export default i18n;
```

#### Frontend: Translation Files

**English Translations** (`website/public/locales/en/auth.json`)

```json
{
  "signup_title": "Create Your Account",
  "signup_subtitle": "Join the Physical AI community to unlock personalized learning",
  "signin_title": "Sign In",
  "signin_subtitle": "Welcome back to your learning journey",
  "step_indicator": "Step {{current}} of {{total}}",
  "email_label": "Email Address",
  "email_placeholder": "you@example.com",
  "password_label": "Password",
  "password_placeholder": "At least 12 characters",
  "confirm_password_label": "Confirm Password",
  "name_label": "Full Name",
  "next_button": "Next",
  "back_button": "Back",
  "complete_button": "Create Account",
  "signin_button": "Sign In",
  "already_have_account": "Already have an account?",
  "dont_have_account": "Don't have an account?",
  "email_verification_sent": "Verification email sent. Please check your inbox.",
  "error_email_exists": "This email is already registered",
  "error_password_mismatch": "Passwords do not match",
  "error_invalid_email": "Please enter a valid email address"
}
```

**Urdu Translations** (`website/public/locales/ur/auth.json`)

```json
{
  "signup_title": "اپنا اکاؤنٹ بنائیں",
  "signup_subtitle": "شخصی سیکھنے کے لیے Physical AI کمیونٹی میں شامل ہوں",
  "signin_title": "سائن ان کریں",
  "signin_subtitle": "اپنے سیکھنے کی سفر میں خوش آمدید",
  "step_indicator": "مرحلہ {{current}} از {{total}}",
  "email_label": "ای میل پتہ",
  "email_placeholder": "you@example.com",
  "password_label": "پاس ورڈ",
  "password_placeholder": "کم از کم 12 حروف",
  "confirm_password_label": "پاس ورڈ کی تصدیق کریں",
  "name_label": "مکمل نام",
  "next_button": "اگلا",
  "back_button": "واپس",
  "complete_button": "اکاؤنٹ بنائیں",
  "signin_button": "سائن ان کریں",
  "already_have_account": "پہلے سے اکاؤنٹ ہے؟",
  "dont_have_account": "اکاؤنٹ نہیں ہے؟",
  "email_verification_sent": "تصدیق ای میل بھیجی گئی۔ براہ کرم اپنے صندوق کو چیک کریں۔",
  "error_email_exists": "یہ ای میل پہلے سے رجسٹرڈ ہے",
  "error_password_mismatch": "پاس ورڈ مماثل نہیں ہیں",
  "error_invalid_email": "براہ کرم ایک درست ای میل پتہ درج کریں"
}
```

**Background Questions** (`website/public/locales/en/forms.json`)

```json
{
  "programming_years_label": "Years of Programming Experience",
  "programming_years_0": "No experience",
  "programming_years_1-3": "1-3 years",
  "programming_years_3-5": "3-5 years",
  "programming_years_5-10": "5-10 years",
  "programming_years_10+": "10+ years",

  "languages_label": "Familiar Programming Languages",
  "languages_help": "Select all that apply",

  "ai_ml_level_label": "AI/ML Experience",
  "ai_ml_none": "None",
  "ai_ml_beginner": "Beginner",
  "ai_ml_intermediate": "Intermediate",
  "ai_ml_advanced": "Advanced",

  "ros_experience_label": "ROS Experience",
  "ros_none": "No experience",
  "ros_basic": "Basic",
  "ros_intermediate": "Intermediate",
  "ros_advanced": "Advanced",

  "hardware_platforms_label": "Hardware Platforms",
  "hardware_platforms_help": "Select all that apply",

  "humanoid_experience_label": "Humanoid Robot Experience",
  "humanoid_none": "No experience",
  "humanoid_simulation": "Simulation only",
  "humanoid_physical": "Physical hardware",
  "humanoid_research": "Research level",

  "simulation_tools_label": "Simulation Tools",
  "simulation_tools_help": "Select all that apply",

  "gpu_access_label": "Do you have GPU access?",
  "hardware_specs_label": "Your Hardware Specifications",
  "hardware_specs_basic": "Basic (CPU only)",
  "hardware_specs_standard": "Standard (GPU, 16GB RAM)",
  "hardware_specs_high-performance": "High-performance (Multiple GPUs, 32GB+)"
}
```

#### Frontend: Multi-Step Form Component (`website/src/components/auth/MultiStepForm.tsx`)

```typescript
import React, { useState } from "react";
import { useForm } from "react-hook-form";
import { useTranslation } from "react-i18next";
import SignupStep1 from "./SignupStep1";
import SignupStep2 from "./SignupStep2";
import SignupStep3 from "./SignupStep3";
import SignupStep4 from "./SignupStep4";
import styles from "../../pages/auth.module.css";

const STEPS = [
  { id: 1, label: "Credentials" },
  { id: 2, label: "Software Background" },
  { id: 3, label: "Robotics Background" },
  { id: 4, label: "Hardware Access" },
];

export const MultiStepForm: React.FC = () => {
  const { t } = useTranslation(["auth", "forms"]);
  const [currentStep, setCurrentStep] = useState(1);
  const [formData, setFormData] = useState({});

  const control = useFormContext().control;
  const errors = useFormContext().formState.errors;

  const handleNext = () => {
    if (currentStep < STEPS.length) {
      setCurrentStep(currentStep + 1);
    }
  };

  const handleBack = () => {
    if (currentStep > 1) {
      setCurrentStep(currentStep - 1);
    }
  };

  const handleSubmit = async (data) => {
    // Merge all form data and submit
    const allData = { ...formData, ...data };
    await submitSignup(allData);
  };

  return (
    <form onSubmit={handleSubmit}>
      {/* Progress Indicator */}
      <div className={styles.progressContainer}>
        <div className={styles.progressBar}>
          <div
            className={styles.progressFill}
            style={{ width: `${(currentStep / STEPS.length) * 100}%` }}
          />
        </div>
        <p className={styles.stepIndicator}>
          {t("auth:step_indicator", { current: currentStep, total: STEPS.length })}
        </p>
      </div>

      {/* Step Content */}
      {currentStep === 1 && <SignupStep1 control={control} errors={errors} />}
      {currentStep === 2 && <SignupStep2 control={control} errors={errors} />}
      {currentStep === 3 && <SignupStep3 control={control} errors={errors} />}
      {currentStep === 4 && <SignupStep4 control={control} errors={errors} />}

      {/* Navigation Buttons */}
      <div className={styles.buttonGroup}>
        {currentStep > 1 && (
          <button type="button" onClick={handleBack} className={styles.secondaryButton}>
            {t("auth:back_button")}
          </button>
        )}

        {currentStep < STEPS.length ? (
          <button type="button" onClick={handleNext} className={styles.primaryButton}>
            {t("auth:next_button")}
          </button>
        ) : (
          <button type="submit" className={styles.primaryButton}>
            {t("auth:complete_button")}
          </button>
        )}
      </div>
    </form>
  );
};
```

#### Frontend: Language Selector (`website/src/components/LanguageSelector.tsx`)

```typescript
import React from "react";
import { useTranslation } from "react-i18next";
import styles from "./LanguageSelector.module.css";

export const LanguageSelector: React.FC = () => {
  const { i18n } = useTranslation();

  const handleLanguageChange = (lang: string) => {
    i18n.changeLanguage(lang);
    localStorage.setItem("language_preference", lang);
  };

  return (
    <div className={styles.languageSelector}>
      <button
        onClick={() => handleLanguageChange("en")}
        className={i18n.language === "en" ? styles.active : ""}
      >
        English
      </button>
      <button
        onClick={() => handleLanguageChange("ur")}
        className={i18n.language === "ur" ? styles.active : ""}
      >
        اردو
      </button>
    </div>
  );
};
```

### Deliverables (Phase 2)
- ✓ i18n configuration and setup
- ✓ English translation files (auth.json, forms.json, errors.json, chat.json)
- ✓ Urdu translation files (matching English structure)
- ✓ MultiStepForm component with progress indicator
- ✓ Individual step components (Step1-4)
- ✓ LanguageSelector component
- ✓ Form styling and responsive design
- ✓ Keyboard navigation and accessibility

### Success Criteria
- All form labels, placeholders, buttons render in selected language
- Language toggle works on all pages
- Language preference persists in localStorage and syncs to DB
- Step-back navigation works smoothly
- Form validation messages appear in selected language
- Component renders in under 300ms (measured with React DevTools Profiler)

---

## Phase 3: Backend Signup/Signin Flow & Profile Integration

### Objectives
- Create REST endpoints for bilingual signup/signin
- Integrate Better Auth for password hashing and session management
- Store user profile data in database
- Log authentication events for audit trail
- Implement email verification flow (optional)

### Architecture Decisions

**AD3.1: Multi-Step Signup Validation**
- **Decision**: Validate all 4 steps on backend before committing
- **Rationale**: Prevents invalid data storage, ensures consistency
- **Validation Sequence**: Email uniqueness → Password strength → Profile data consistency
- **Error Reporting**: Return field-level errors in response for client-side UI feedback

**AD3.2: Profile Data Atomicity**
- **Decision**: Create both user + profile in a single transaction
- **Rationale**: Prevents orphaned records, maintains data consistency
- **Rollback Strategy**: If profile creation fails, entire signup fails (user not created)

**AD3.3: Session Management with Better Auth**
- **Decision**: Use Better Auth for session tokens and cookie management
- **Rationale**: Securely manage sessions, automatic expiration, refresh tokens
- **Client Storage**: Session token in httpOnly cookie (set by server), not localStorage

### Components & Responsibilities

#### Backend: Signup Endpoint (`backend/api/routes/auth.py`)

```python
from fastapi import APIRouter, HTTPException, Depends
from ..models import SignupRequest, SigninRequest, AuthResponse
from ..services import UserService, AuthService, AuditService
from ..database import get_db
import uuid

router = APIRouter(prefix="/api/auth", tags=["auth"])

@router.post("/signup")
async def signup(
    request: SignupRequest,
    db = Depends(get_db),
    user_service: UserService = Depends(),
    auth_service: AuthService = Depends(),
    audit_service: AuditService = Depends(),
):
    """
    Multi-step signup with profile creation.

    Request:
    {
      "email": "user@example.com",
      "password": "secure_password_12chars+",
      "name": "User Name",
      "language_preference": "en",
      "profile": {
        "programming_years": "10+",
        "languages": ["Python", "C++"],
        "ai_ml_level": "advanced",
        "ros_experience": "intermediate",
        "hardware_platforms": ["Jetson"],
        "humanoid_experience": "research",
        "simulation_tools": ["Gazebo"],
        "gpu_access": true,
        "hardware_specs": "high-performance",
        "lab_equipment_access": []
      }
    }
    """

    try:
        # Validate inputs
        if not request.email or "@" not in request.email:
            raise HTTPException(status_code=400, detail={"field": "email", "error": "Invalid email"})

        if len(request.password) < 12:
            raise HTTPException(status_code=400, detail={"field": "password", "error": "Password must be 12+ characters"})

        # Check email uniqueness
        existing = await user_service.get_user_by_email(request.email)
        if existing:
            raise HTTPException(status_code=409, detail={"field": "email", "error": "Email already registered"})

        # Create user with Better Auth
        user_id = str(uuid.uuid4())
        password_hash = auth_service.hash_password(request.password)

        user = await user_service.create_user(
            id=user_id,
            email=request.email,
            name=request.name,
            password_hash=password_hash,
            email_verified=False,
        )

        # Create profile
        profile = await user_service.create_user_profile(
            user_id=user_id,
            profile_data={
                "programming_years": request.profile.programming_years,
                "languages": request.profile.languages,
                "ai_ml_level": request.profile.ai_ml_level,
                "ros_experience": request.profile.ros_experience,
                "hardware_platforms": request.profile.hardware_platforms,
                "humanoid_experience": request.profile.humanoid_experience,
                "simulation_tools": request.profile.simulation_tools,
                "gpu_access": request.profile.gpu_access,
                "hardware_specs": request.profile.hardware_specs,
                "lab_equipment_access": request.profile.lab_equipment_access,
                "language_preference": request.language_preference or "en",
            }
        )

        # Create session with Better Auth
        session_token = auth_service.create_session(user_id)

        # Log audit event
        await audit_service.log_event(
            user_id=user_id,
            action="signup",
            resource="user",
            details={"email": request.email, "language": request.language_preference},
        )

        # Send verification email if configured
        if os.getenv("ENABLE_EMAIL_VERIFICATION") == "true":
            await auth_service.send_verification_email(user.email)

        return AuthResponse(
            user_id=user.id,
            email=user.email,
            name=user.name,
            session_token=session_token,
            email_verified=user.email_verified,
        )

    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Signup error: {e}")
        raise HTTPException(status_code=500, detail="Signup failed")


@router.post("/signin")
async def signin(
    request: SigninRequest,
    user_service: UserService = Depends(),
    auth_service: AuthService = Depends(),
    audit_service: AuditService = Depends(),
):
    """
    Signin with email and password.

    Request:
    {
      "email": "user@example.com",
      "password": "secure_password"
    }
    """

    try:
        # Get user
        user = await user_service.get_user_by_email(request.email)
        if not user:
            raise HTTPException(status_code=401, detail="Invalid email or password")

        # Verify password
        if not auth_service.verify_password(request.password, user.password_hash):
            raise HTTPException(status_code=401, detail="Invalid email or password")

        # Get profile for context
        profile = await user_service.get_user_profile(user.id)

        # Create session
        session_token = auth_service.create_session(user.id)

        # Log audit event
        await audit_service.log_event(
            user_id=user.id,
            action="signin",
            resource="user",
            details={"email": user.email},
        )

        return AuthResponse(
            user_id=user.id,
            email=user.email,
            name=user.name,
            session_token=session_token,
            profile=profile,
        )

    except Exception as e:
        logger.error(f"Signin error: {e}")
        raise HTTPException(status_code=500, detail="Signin failed")


@router.get("/session")
async def get_session(
    user_id: str = Depends(get_current_user_id),
    user_service: UserService = Depends(),
):
    """Get current session user info."""
    user = await user_service.get_user_by_id(user_id)
    profile = await user_service.get_user_profile(user_id)

    return {
        "user": {
            "id": user.id,
            "email": user.email,
            "name": user.name,
        },
        "profile": profile,
    }
```

#### Backend: Better Auth Service (`backend/services/auth_service.py`)

```python
from bcrypt import hashpw, checkpw, gensalt
import os
from datetime import datetime, timedelta
import secrets
from typing import Tuple

class AuthService:
    """Authentication service using Better Auth patterns."""

    def hash_password(self, password: str) -> str:
        """Hash password with bcrypt (12 rounds)."""
        salt = gensalt(rounds=12)
        return hashpw(password.encode("utf-8"), salt).decode("utf-8")

    def verify_password(self, password: str, hash: str) -> bool:
        """Verify password against bcrypt hash."""
        try:
            return checkpw(password.encode("utf-8"), hash.encode("utf-8"))
        except Exception:
            return False

    def create_session(self, user_id: str) -> str:
        """Create session token (httpOnly cookie set by framework)."""
        session_token = secrets.token_urlsafe(32)
        expires_at = datetime.utcnow() + timedelta(days=7)

        # Store in database
        db.execute(
            """
            INSERT INTO sessions (id, user_id, expires_at)
            VALUES (%s, %s, %s)
            """,
            (session_token, user_id, expires_at)
        )

        return session_token

    def verify_session(self, session_token: str) -> str | None:
        """Verify session token and return user_id if valid."""
        result = db.execute(
            """
            SELECT user_id FROM sessions
            WHERE id = %s AND expires_at > NOW()
            """,
            (session_token,)
        ).fetchone()

        return result[0] if result else None

    async def send_verification_email(self, email: str):
        """Send email verification link."""
        # Implementation depends on email service (SendGrid, AWS SES, etc.)
        pass
```

### Deliverables (Phase 3)
- ✓ Signup endpoint with profile creation and transaction handling
- ✓ Signin endpoint with password verification
- ✓ Session management with Better Auth patterns
- ✓ Audit logging for all auth events
- ✓ Email verification flow (optional, configurable)
- ✓ Input validation with field-level error reporting
- ✓ Database transaction safety

### Success Criteria
- Signup endpoint accepts all 4 steps of data
- Profile data stored in database with correct structure
- Passwords hashed with bcrypt (12 rounds)
- Session tokens generated and validated correctly
- Audit logs record signup/signin events with timestamps
- Email verification optional and configurable via env var

---

## Phase 4: Content Gating & Chat Personalization

### Objectives
- Implement content access rules based on AI/ML experience level
- Create middleware to enforce content gating
- Pass user profile context to RAG chatbot
- Filter chat recommendations by user background
- Display upgrade prompts for restricted content

### Architecture Decisions

**AD4.1: Server-Side Content Gating (Security)**
- **Decision**: All content access checks performed on backend, not frontend
- **Rationale**: Frontend checks can be bypassed; backend is authoritative
- **Implementation**: Middleware on protected routes, 403 Forbidden with message in user's language

**AD4.2: Profile Context Injection**
- **Decision**: Include user profile in request context for all API calls
- **Rationale**: Enables personalization without additional database queries
- **Caching**: Cache profile for session duration (5-minute TTL)

**AD4.3: Recommendation Filtering**
- **Decision**: Filter recommendations in chatbot context, not in UI
- **Rationale**: Ensures consistent behavior across interfaces
- **Filter Strategy**: Include user profile in system prompt to RAG chatbot

### Components & Responsibilities

#### Backend: Content Gating Middleware (`backend/api/middleware/gating.py`)

```python
from fastapi import HTTPException, Depends, Request
from ..models import ContentTier
from ..services import UserService

CONTENT_ACCESS_RULES = {
    "beginner": ["basics", "fundamentals"],
    "intermediate": ["basics", "fundamentals", "intermediate", "projects"],
    "advanced": ["all", "including", "capstone", "research"],
    "none": [],
}

async def check_content_access(
    request: Request,
    user_id: str = Depends(get_current_user_id),
    user_service: UserService = Depends(),
):
    """Middleware to check content access based on profile."""

    user_profile = await user_service.get_user_profile(user_id)
    if not user_profile:
        raise HTTPException(status_code=403, detail="Profile not found")

    ai_ml_level = user_profile.ai_ml_level
    allowed_tiers = CONTENT_ACCESS_RULES.get(ai_ml_level, [])

    # Extract requested content tier from route
    content_tier = request.path.split("/")[2]  # /content/{tier}/{page}

    if content_tier not in allowed_tiers:
        user_lang = user_profile.language_preference
        error_msg = t("errors:access_denied", lang=user_lang)

        raise HTTPException(
            status_code=403,
            detail={
                "error": "Access denied",
                "message": error_msg,
                "current_level": ai_ml_level,
                "required_level": get_min_level_for_tier(content_tier),
                "upgrade_link": f"/profile/upgrade?target={content_tier}",
            }
        )

    # Store profile in request for downstream handlers
    request.state.user_profile = user_profile
    return user_profile


@router.get("/content/{tier}/{page}")
async def get_content(
    tier: str,
    page: str,
    profile: UserProfile = Depends(check_content_access),
):
    """Get content with access control."""
    # Content retrieved here is guaranteed accessible to user
    return fetch_content(tier, page)
```

#### Backend: RAG Chatbot Context (`backend/services/rag_chatbot.py`)

```python
class RAGChatbot:
    """RAG chatbot with user profile-aware personalization."""

    async def answer_query_stream(
        self,
        query: str,
        user_id: str,
        mode: str = "full",
        selected_text: str = None,
        top_k: int = 5,
    ):
        """Answer query with user profile context for personalization."""

        # Get user profile
        user_profile = await user_service.get_user_profile(user_id)

        # Build personalization context
        profile_context = self._build_profile_context(user_profile)

        # Retrieve relevant documents
        query_with_context = f"{query}\n\n[User Context: {profile_context}]"

        documents = await self.vector_store.retrieve(
            query=query_with_context,
            top_k=top_k,
            filter=self._get_content_filter(user_profile.ai_ml_level),
        )

        # Generate response with personalization
        system_prompt = f"""You are an expert tutor for Physical AI and Robotics.

        User Background:
        {profile_context}

        Tailor your response to match the user's experience level. For advanced users, include
        research papers and cutting-edge techniques. For beginners, focus on fundamentals.
        Acknowledge their background: "Since you have {user_profile.ai_ml_level} AI/ML experience..."
        """

        async for chunk in self.openrouter_client.stream_completion(
            system_prompt=system_prompt,
            user_message=query,
            context_documents=documents,
        ):
            yield chunk

    def _build_profile_context(self, profile: UserProfile) -> str:
        """Build human-readable profile context for system prompt."""
        return f"""
        - Programming Experience: {profile.programming_years} years
        - Languages: {', '.join(profile.languages) or 'None specified'}
        - AI/ML Level: {profile.ai_ml_level}
        - ROS Experience: {profile.ros_experience}
        - Hardware: {', '.join(profile.hardware_platforms) or 'No specific hardware'}
        - Humanoid Robotics: {profile.humanoid_experience}
        - GPU Access: {'Yes' if profile.gpu_access else 'No'}
        """

    def _get_content_filter(self, ai_ml_level: str) -> dict:
        """Get vector store filter for content access."""
        return {
            "access_level": {
                "beginner": ["fundamentals"],
                "intermediate": ["fundamentals", "intermediate"],
                "advanced": ["fundamentals", "intermediate", "advanced", "research"],
                "none": [],
            }.get(ai_ml_level, [])
        }
```

#### Frontend: Content Gating Display (`website/src/components/ContentGate.tsx`)

```typescript
import React from "react";
import { useAuthContext } from "../context/AuthContext";
import { useTranslation } from "react-i18next";

interface ContentGateProps {
  requiredLevel: "beginner" | "intermediate" | "advanced";
  children: React.ReactNode;
}

export const ContentGate: React.FC<ContentGateProps> = ({ requiredLevel, children }) => {
  const { user, profile } = useAuthContext();
  const { t } = useTranslation("auth");

  if (!user) {
    return (
      <div className="content-gate-message">
        <p>{t("please_sign_in")}</p>
        <a href="/signin">Sign In</a>
      </div>
    );
  }

  const levelHierarchy = { beginner: 0, intermediate: 1, advanced: 2 };
  const userLevel = levelHierarchy[profile?.ai_ml_level] || -1;
  const requiredLevelValue = levelHierarchy[requiredLevel];

  if (userLevel < requiredLevelValue) {
    return (
      <div className="content-gate-message">
        <h3>🔒 Advanced Content</h3>
        <p>
          {t("access_denied_message", {
            level: requiredLevel,
            currentLevel: profile?.ai_ml_level,
          })}
        </p>
        <a href="/profile/upgrade">
          {t("upgrade_profile")}
        </a>
      </div>
    );
  }

  return <>{children}</>;
};
```

#### Frontend: Profile Page (`website/src/pages/profile.tsx`)

```typescript
import React, { useEffect, useState } from "react";
import { useAuthContext } from "../context/AuthContext";
import { useTranslation } from "react-i18next";
import { apiClient } from "../services/apiClient";

export default function ProfilePage() {
  const { user, profile } = useAuthContext();
  const { t, i18n } = useTranslation("forms");
  const [isEditing, setIsEditing] = useState(false);

  if (!user) {
    return <div>Please sign in to view your profile</div>;
  }

  const contentAccessLevel = {
    beginner: t("content_access_basic"),
    intermediate: t("content_access_intermediate"),
    advanced: t("content_access_all"),
    none: t("content_access_none"),
  };

  return (
    <div className="profile-page">
      <h1>{t("my_profile")}</h1>

      {/* User Info */}
      <section className="profile-section">
        <h2>{t("account_info")}</h2>
        <p>Email: {user.email}</p>
        <p>Name: {user.name}</p>
        <p>
          Content Access: <span className="access-badge">{contentAccessLevel[profile?.ai_ml_level]}</span>
        </p>
      </section>

      {/* Background Info */}
      <section className="profile-section">
        <h2>{t("background_info")}</h2>
        <p>
          Programming Experience: <strong>{profile?.programming_years}</strong>
        </p>
        <p>
          Languages: <strong>{profile?.languages.join(", ")}</strong>
        </p>
        <p>
          AI/ML Level: <strong>{profile?.ai_ml_level}</strong>
        </p>
        <p>
          ROS Experience: <strong>{profile?.ros_experience}</strong>
        </p>
        <p>
          Hardware: <strong>{profile?.hardware_platforms.join(", ")}</strong>
        </p>
      </section>

      {/* Upgrade Path */}
      {profile?.ai_ml_level !== "advanced" && (
        <section className="upgrade-section">
          <h2>🎯 Upgrade Your Access</h2>
          <p>To unlock advanced content, build your skills in:</p>
          <ul>
            <li>Advanced AI/ML projects and research</li>
            <li>Physical humanoid robotics</li>
            <li>Research-level publications</li>
          </ul>
          <p>Update your profile to reflect your latest experience.</p>
          <button onClick={() => setIsEditing(true)}>Edit Profile</button>
        </section>
      )}
    </div>
  );
}
```

### Deliverables (Phase 4)
- ✓ Content gating middleware with access rules
- ✓ 403 Forbidden responses with bilingual error messages
- ✓ RAG chatbot context with user profile
- ✓ Profile-aware recommendation filtering
- ✓ Content gate UI component for frontend
- ✓ User profile management page
- ✓ Upgrade prompts and guidance

### Success Criteria
- Users cannot access content above their AI/ML level (403 Forbidden)
- Error messages in user's language preference
- Chat recommendations filtered by user background
- Chat responses acknowledge user experience level
- Profile data retrieved in under 100ms
- Beginner/intermediate/advanced users see different content tiers

---

## Phase 5: Testing, Security Validation & Deployment

### Objectives
- Complete end-to-end testing of bilingual signup/signin flow
- Security assessment (OWASP Top 10)
- Performance testing and optimization
- Production deployment and monitoring

### Testing Strategy

#### 1. Unit Tests (`backend/tests/test_auth.py`)

```python
import pytest
from backend.services.auth_service import AuthService
from backend.services.user_service import UserService

@pytest.fixture
def auth_service():
    return AuthService()

def test_password_hashing(auth_service):
    """Test bcrypt password hashing."""
    password = "secure_password_12chars"
    hash = auth_service.hash_password(password)

    assert hash != password
    assert auth_service.verify_password(password, hash)
    assert not auth_service.verify_password("wrong_password", hash)

def test_profile_creation(user_service):
    """Test user profile creation with all fields."""
    user_id = "test-user-123"
    profile_data = {
        "programming_years": "10+",
        "languages": ["Python", "C++"],
        "ai_ml_level": "advanced",
        # ... other fields
    }

    profile = user_service.create_user_profile(user_id, profile_data)
    assert profile.user_id == user_id
    assert profile.ai_ml_level == "advanced"
```

#### 2. Integration Tests (`backend/tests/test_signup_flow.py`)

```python
import pytest
from httpx import AsyncClient
from backend.app import app

@pytest.mark.asyncio
async def test_complete_signup_flow():
    """Test full 4-step signup process."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/api/auth/signup",
            json={
                "email": "new_user@example.com",
                "password": "secure_password_12chars",
                "name": "New User",
                "language_preference": "en",
                "profile": {
                    "programming_years": "10+",
                    "languages": ["Python"],
                    "ai_ml_level": "advanced",
                    # ... all profile fields
                }
            }
        )

        assert response.status_code == 200
        data = response.json()
        assert data["user_id"]
        assert data["session_token"]

@pytest.mark.asyncio
async def test_bilingual_validation_errors():
    """Test validation errors in both languages."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # English error
        response = await client.post(
            "/api/auth/signup?lang=en",
            json={"email": "invalid", "password": "short"}
        )

        assert response.status_code == 400
        error_msg = response.json()["detail"]
        assert "Invalid email" in error_msg or "invalid" in error_msg.lower()
```

#### 3. E2E Tests (`website/tests/auth.e2e.test.ts`)

```typescript
import { test, expect } from "@playwright/test";

test.describe("Bilingual Signup Flow", () => {
  test("English signup with background questions", async ({ page }) => {
    await page.goto("/signup");

    // Step 1: Credentials
    await page.fill('input[name="email"]', "user@example.com");
    await page.fill('input[name="password"]', "secure_password_12chars");
    await page.fill('input[name="confirm_password"]', "secure_password_12chars");
    await page.fill('input[name="name"]', "Test User");
    await page.click("button:has-text('Next')");

    // Step 2: Software Background
    await page.selectOption('select[name="programming_years"]', "10+");
    await page.check('input[value="Python"]');
    await page.click("button:has-text('Next')");

    // Step 3: Robotics Background
    await page.check('input[value="Gazebo"]');
    await page.click("button:has-text('Next')");

    // Step 4: Hardware Access
    await page.check('input[name="gpu_access"]');
    await page.click("button:has-text('Create Account')");

    // Verify success
    await expect(page).toHaveURL(/\/(dashboard|home)/);
  });

  test("Urdu signup with language toggle", async ({ page }) => {
    await page.goto("/signup");

    // Toggle to Urdu
    await page.click("button:has-text('اردو')");

    // Verify labels in Urdu
    const emailLabel = await page.textContent('label:first-of-type');
    expect(emailLabel).toContain("ای میل");

    // Complete signup in Urdu
    // ... form filling
  });

  test("Content gating after signup", async ({ page }) => {
    // Login as beginner user
    await loginAs("beginner");

    // Try to access advanced content
    await page.goto("/content/capstone");

    // Verify access denied message
    await expect(page.locator(".content-gate-message")).toBeVisible();
  });
});
```

#### 4. Security Tests

```python
# Test OWASP Top 10 vulnerabilities

def test_sql_injection_prevention():
    """Verify SQL injection is prevented."""
    # Drizzle ORM with parameterized queries prevents this
    user = db.select(users).where(users.email == "admin'; DROP TABLE users; --")
    assert user is None

def test_xss_prevention():
    """Verify XSS is prevented in form inputs."""
    # React escapes by default; verify on backend too
    response = client.post("/api/auth/signup", json={
        "email": "<script>alert('xss')</script>@example.com",
        "name": "<img src=x onerror=alert('xss')>",
    })
    assert response.status_code == 400

def test_csrf_token_validation():
    """Verify CSRF tokens are validated."""
    # FastAPI with CORS + same-site cookies provides CSRF protection
    response = client.post("/api/auth/signup", cookies={})
    assert response.status_code == 403 or "CSRF" in str(response.text)

def test_password_strength():
    """Verify minimum password requirements."""
    weak_passwords = [
        "short",
        "123456789",
        "abcdefg1234",  # Only 11 chars
    ]
    for pwd in weak_passwords:
        response = client.post("/api/auth/signup", json={
            "email": "user@example.com",
            "password": pwd,
        })
        assert response.status_code == 400
        assert "12" in response.json()["detail"]

def test_bcrypt_hashing():
    """Verify passwords are hashed with bcrypt."""
    # Cannot directly test hash, but verify different inputs produce different hashes
    hash1 = auth_service.hash_password("password1")
    hash2 = auth_service.hash_password("password1")
    assert hash1 != hash2  # Different salts
    assert auth_service.verify_password("password1", hash1)
    assert auth_service.verify_password("password1", hash2)
```

#### 5. Performance Tests

```python
import time

def test_signup_response_time():
    """Verify signup completes in under 1 second."""
    start = time.time()
    response = client.post("/api/auth/signup", json={...})
    elapsed = time.time() - start
    assert elapsed < 1.0, f"Signup took {elapsed}s, expected < 1s"

def test_profile_retrieval_latency():
    """Verify profile retrieval under 100ms."""
    start = time.time()
    profile = db.query(userProfiles).filter(...).first()
    elapsed = time.time() - start
    assert elapsed < 0.1, f"Profile query took {elapsed}s, expected < 100ms"

def test_language_switch_performance():
    """Verify language switch renders in under 300ms."""
    # Browser-based test with React DevTools Profiler
    # Expected: Initial render < 50ms, re-render with language change < 300ms
```

#### 6. Bilingual Validation Tests

```python
def test_all_labels_translated():
    """Verify no hardcoded English text in components."""
    # Use i18n validation tool to check all translation files
    en_keys = set(en_translations.keys())
    ur_keys = set(ur_translations.keys())

    assert en_keys == ur_keys, f"Missing translations: {en_keys ^ ur_keys}"

def test_error_messages_bilingual():
    """Verify error messages appear in correct language."""
    # For English
    response = client.post("/api/auth/signup?lang=en", json={...})
    assert "Invalid email" in response.json()["detail"]

    # For Urdu
    response = client.post("/api/auth/signup?lang=ur", json={...})
    assert "ای میل" in response.json()["detail"]  # Urdu for email
```

### Security Assessment (OWASP Top 10)

| Vulnerability | Status | Mitigation |
|---|---|---|
| A01:2021 Broken Access Control | ✓ Mitigated | Server-side content gating, role-based access checks |
| A02:2021 Cryptographic Failures | ✓ Mitigated | bcrypt hashing (12 rounds), HTTPS only, encrypted fields at rest |
| A03:2021 Injection | ✓ Mitigated | Drizzle ORM with parameterized queries, input validation |
| A04:2021 Insecure Design | ✓ Mitigated | Better Auth framework, secure session management |
| A05:2021 Security Misconfiguration | ✓ Mitigated | Environment-based config, CORS restrictions, secure headers |
| A06:2021 Vulnerable & Outdated Components | ✓ Monitored | Dependabot, regular npm/pip audits |
| A07:2021 Identification & Authentication Failures | ✓ Mitigated | Better Auth session management, email verification |
| A08:2021 Software & Data Integrity Failures | ✓ Mitigated | Package integrity checks, signed commits |
| A09:2021 Logging & Monitoring Failures | ✓ Mitigated | Audit logs for all auth events, error tracking |
| A10:2021 Server-Side Request Forgery (SSRF) | ✓ Safe | No external API calls from user input |

### Deployment Checklist

#### Pre-Deployment
- [ ] All tests passing (unit, integration, E2E)
- [ ] Security scan complete (npm audit, pip audit)
- [ ] Performance tests show < 1s form submission
- [ ] All translations complete and validated
- [ ] Database migration tested in staging
- [ ] Environment variables configured (BETTER_AUTH_SECRET, ENABLE_EMAIL_VERIFICATION, etc.)
- [ ] Rate limiting configured (signup endpoint, signin endpoint)
- [ ] Error tracking configured (Sentry, DataDog, etc.)

#### Deployment
- [ ] Database migration deployed to Neon
- [ ] Backend deployed to production
- [ ] Frontend built and deployed
- [ ] Email verification service configured (if enabled)
- [ ] Monitoring and alerting enabled
- [ ] Runbook for common issues created

#### Post-Deployment
- [ ] Monitor auth success rate (target: > 99%)
- [ ] Monitor signup completion rate (target: > 80%)
- [ ] Monitor form validation error rates
- [ ] Audit logs populated correctly
- [ ] Chat personalization working (test with different user profiles)
- [ ] Content gating enforced (test beginner can't access advanced)

### Deliverables (Phase 5)
- ✓ Complete test suite (unit, integration, E2E)
- ✓ Security assessment report (OWASP Top 10)
- ✓ Performance test results
- ✓ Bilingual validation tests
- ✓ Database migration for production
- ✓ Deployment guide and runbooks
- ✓ Monitoring and alerting configuration
- ✓ Rollback strategy documented

### Success Criteria
- All automated tests passing (>95% code coverage)
- Security assessment shows no critical vulnerabilities
- Signup completes in under 3 minutes
- Form response time under 1 second
- All 35+ acceptance criteria met
- Bilingual UI fully functional in English and Urdu
- Content gating enforced on all protected routes

---

## Integration Points & Data Flow

### 1. Authentication Flow

```
[Frontend Signup]
  → POST /api/auth/signup (email, password, profile, language)
    → Better Auth hashes password (bcrypt 12 rounds)
    → UserService creates user + profile transaction
    → AuditService logs "signup" event
    → Session token generated
  ← 200 OK with session_token, user_id
  → localStorage.setItem("session_token", token)
  → Redirect to /dashboard

[Frontend Chat]
  → Authorization header includes session token
  → GET /api/session (verify token, get user)
  → Retrieve user profile
  ← 200 OK with user data + profile
  → Pass profile to RAG chatbot context
  → Filter recommendations by ai_ml_level
```

### 2. Content Gating Flow

```
[User requests /content/capstone]
  → Frontend makes request with session token
  → Backend middleware (check_content_access)
    → Verify session token
    → Load user profile
    → Check ai_ml_level against content_tier
  → If access denied:
    ← 403 Forbidden with error in user's language
    → Show "Upgrade your profile" link
  → If allowed:
    ← 200 OK with content
    → AuditService logs "content_access" event
```

### 3. Chat Personalization Flow

```
[User asks question in chat]
  → Frontend sends query with session token
  → POST /api/chat-stream
    → get_current_user_id middleware extracts user_id from token
    → UserService retrieves profile (cached for 5 mins)
    → RAGChatbot._build_profile_context() creates context
    → System prompt includes profile context
    → Vector store filters by access_level
    → Stream response with personalized recommendations
  ← Streaming JSON responses with citations
    → Frontend renders citations with source links
    → Content marked as filtered by user level
```

---

## Risk Analysis & Mitigation

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|-----------|
| Better Auth API changes | High | Low | Pin version, monitor releases, maintain wrapper |
| Database migration failure | High | Low | Test on staging, full backup before deploy, rollback script |
| Bilingual translation gaps | Medium | Medium | Validation tool, peer review process, incomplete flag in UI |
| Email service unavailable | Medium | Low | Queue verification emails, retry logic, admin bypass |
| Performance degradation | Medium | Medium | Monitor p95 latency, cache profile (5 min TTL), optimize queries |
| Session token leaks | High | Very Low | httpOnly cookies, HTTPS only, short expiration (7 days) |
| Content gating bypasses | High | Very Low | Server-side checks, no client-side gating, audit logs |

---

## Technology Stack Summary

| Layer | Technology | Purpose |
|-------|-----------|---------|
| **Backend** | FastAPI + Better Auth | REST API, authentication, session management |
| **Database** | PostgreSQL (Neon) + Drizzle ORM | Data persistence, type-safe queries |
| **Frontend** | React + Docusaurus v3+ | UI framework, documentation |
| **Bilingual** | react-i18next | Translation management |
| **Auth** | Better Auth | Password hashing (bcrypt), session tokens, email verification |
| **Validation** | Pydantic (backend), react-hook-form (frontend) | Input validation |
| **Testing** | pytest, Playwright, Jest | Automated testing |
| **Monitoring** | Audit logs, error tracking | Compliance and debugging |

---

## Success Metrics & KPIs

### Adoption Metrics
- Signup completion rate > 80%
- Avg signup time < 3 minutes
- Return user rate > 60%

### Quality Metrics
- Form validation error rate < 5%
- Signup success rate > 99%
- Chat response latency p95 < 3 seconds

### Security Metrics
- 100% passwords hashed with bcrypt
- 0 security vulnerabilities (critical)
- Audit logs 100% coverage of auth events

### Bilingual Metrics
- 100% of text translated (English/Urdu)
- Translation validation automated
- Urdu user satisfaction > 80%

---

## Next Steps

1. **Review & Approval**: Stakeholder review of plan
2. **Task Generation**: `/sp.tasks` to create detailed implementation tasks
3. **Implementation**: Execute phases 1-5 in order
4. **ADR Documentation**: `/sp.adr` for significant decisions
5. **Testing & Deployment**: Execute test plan and deploy to production

---

**Plan Version**: 1.0
**Created**: 2025-12-18
**Status**: Ready for Implementation
