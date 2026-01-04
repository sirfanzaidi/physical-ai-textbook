---
title: "004 - Bilingual Authentication Extension - Implementation Tasks"
feature_id: "004-bilingual-auth-extension"
version: "1.0"
created_date: "2025-12-18"
status: "Ready for Implementation"
---

# Implementation Tasks: Bilingual Authentication Extension

## Overview

This document contains **42 prioritized, testable tasks** organized by phase and dependency. Each task specifies:
- Clear acceptance criteria
- Bilingual (English/Urdu) output requirements
- Test cases
- Estimated story points
- Dependencies

**Total Effort**: ~160 story points across 5 phases

---

## Phase 1: Database & Backend Infrastructure (20 tasks, ~35 SP)

### Task 1.1: Create Drizzle ORM Schema Definition

**Description**: Define complete database schema using Drizzle ORM with enums, tables, and relationships for user authentication and profiles.

**Acceptance Criteria**:
- [ ] File created: `backend/database/schema.ts`
- [ ] All enums defined (programming_years, ai_ml_level, ros_experience, humanoid_experience, hardware_specs, language_preference)
- [ ] `users` table defined with id, email, name, emailVerified, createdAt, updatedAt
- [ ] `user_profiles` table defined with all 18 profile fields
- [ ] `sessions` table defined for Better Auth compatibility
- [ ] `audit_logs` table defined with action, resource, details, timestamps
- [ ] All foreign keys and constraints defined
- [ ] TypeScript types exported for use in services
- [ ] Drizzle schema compiles without errors

**Test Case**:
```typescript
import { users, userProfiles, sessions, auditLogs } from "../schema";

// Schema imports without errors
expect(users).toBeDefined();
expect(userProfiles).toBeDefined();
expect(sessions).toBeDefined();
expect(auditLogs).toBeDefined();

// Column types are correct
expect(userProfiles.aiMlLevel.type).toBe("enum");
expect(userProfiles.languages.type).toBe("array");
```

**Bilingual Output**: N/A (TypeScript schema)

**Story Points**: 5

**Dependencies**: None

---

### Task 1.2: Create Database Migration SQL

**Description**: Write SQL migration file to create all tables, enums, indexes, and constraints in PostgreSQL.

**File**: `backend/database/migrations/002_bilingual_auth_schema.sql`

**Acceptance Criteria**:
- [ ] All enums created with correct values
- [ ] All 4 tables created with correct columns and types
- [ ] ARRAY columns use PostgreSQL ARRAY type (not JSON)
- [ ] Foreign key constraints with CASCADE delete
- [ ] Indexes created for performance (user_id, ai_ml_level, session expiry, audit action)
- [ ] Unique constraint on user_profiles.user_id (one profile per user)
- [ ] Migration is idempotent (safe to re-run)
- [ ] Migration includes rollback (DROP statements in reverse order)

**Test Case**:
```sql
-- Verify schema after migration
SELECT * FROM information_schema.tables WHERE table_name LIKE 'user%';
-- Should return: users, user_profiles

SELECT COUNT(*) FROM information_schema.table_constraints
WHERE constraint_type = 'FOREIGN KEY' AND table_name = 'user_profiles';
-- Should return: 1 (user_id -> users)

-- Verify enums exist
SELECT enum_range(NULL::ai_ml_level);
-- Should return: (none,beginner,intermediate,advanced)
```

**Bilingual Output**: SQL comments include English descriptions

**Story Points**: 3

**Dependencies**: Task 1.1

---

### Task 1.3: Set Up Better Auth Configuration

**Description**: Configure Better Auth framework with password hashing, session management, and email verification.

**File**: `backend/auth/better_auth_config.ts` (TypeScript, even though backend is Python, this is config)

**Acceptance Criteria**:
- [ ] Better Auth initialized with PostgreSQL adapter
- [ ] Password requirements: minimum 12 characters
- [ ] Bcrypt hashing configured (12 rounds)
- [ ] Session TTL set to 7 days
- [ ] Email verification optional (configurable via `ENABLE_EMAIL_VERIFICATION` env var)
- [ ] CORS configured to trust frontend URL
- [ ] Session cookie marked as httpOnly and secure
- [ ] Audit logging hook configured
- [ ] All required environment variables documented

**Env Variables Required**:
```bash
BETTER_AUTH_SECRET=<random_32_char_string>
DATABASE_URL=postgresql://user:pass@host/db
FRONTEND_URL=https://frontend.example.com
BACKEND_URL=https://api.example.com
ENABLE_EMAIL_VERIFICATION=false  # or true
```

**Test Case**:
```python
from backend.auth.better_auth_config import auth

# Config loads without errors
assert auth is not None
assert auth.config.secret != ""
assert auth.config.password_min_length == 12

# Session config
assert auth.config.session_expires_in == 60*60*24*7
```

**Bilingual Output**: N/A (Config file)

**Story Points**: 5

**Dependencies**: None

---

### Task 1.4: Create User Service with Drizzle ORM

**Description**: Implement UserService class with methods for user and profile CRUD operations using Drizzle ORM.

**File**: `backend/services/user_service.ts`

**Acceptance Criteria**:
- [ ] `createUser(email, name, passwordHash)` - returns User
- [ ] `getUserById(userId)` - returns User or null
- [ ] `getUserByEmail(email)` - returns User or null
- [ ] `createUserProfile(userId, profileData)` - returns UserProfile
- [ ] `getUserProfile(userId)` - returns UserProfile or null
- [ ] `updateUserProfile(userId, profileData)` - returns updated profile
- [ ] `logAuditEvent(userId, action, resource, details)` - logs to audit_logs table
- [ ] All methods use Drizzle ORM with parameterized queries
- [ ] Transactions used for multi-table operations (user + profile creation)
- [ ] Error handling with proper exception types
- [ ] Type-safe return values using Drizzle schema types

**Test Case**:
```typescript
import { UserService } from "../services/user_service";

const service = new UserService(db);

// Create user
const user = await service.createUser("test@example.com", "Test User", "hashed_pw");
expect(user.id).toBeDefined();
expect(user.email).toBe("test@example.com");

// Create profile
const profile = await service.createUserProfile(user.id, {
  programmingYears: "10+",
  languages: ["Python"],
  aiMlLevel: "advanced",
  // ... other fields
});
expect(profile.userId).toBe(user.id);
expect(profile.languages).toContain("Python");

// Get profile
const retrieved = await service.getUserProfile(user.id);
expect(retrieved.aiMlLevel).toBe("advanced");
```

**Bilingual Output**: N/A (Backend service)

**Story Points**: 8

**Dependencies**: Task 1.1, 1.2, 1.3

---

### Task 1.5: Create Auth Service (Password & Session)

**Description**: Implement authentication service with password hashing, verification, and session management.

**File**: `backend/services/auth_service.ts`

**Acceptance Criteria**:
- [ ] `hashPassword(password)` - uses bcrypt with 12 rounds
- [ ] `verifyPassword(password, hash)` - returns boolean
- [ ] `createSession(userId)` - generates session token and stores in DB
- [ ] `verifySession(sessionToken)` - returns userId or null
- [ ] `refreshSession(sessionToken)` - extends session expiry
- [ ] `revokeSession(sessionToken)` - deletes session
- [ ] `sendVerificationEmail(email)` - implementation stub (configurable)
- [ ] All password operations use bcrypt (not plain text comparisons)
- [ ] Session tokens are cryptographically random (32 bytes minimum)
- [ ] No passwords logged in error messages

**Test Case**:
```typescript
import { AuthService } from "../services/auth_service";

const service = new AuthService();

// Password hashing
const password = "secure_password_12chars";
const hash1 = service.hashPassword(password);
const hash2 = service.hashPassword(password);

expect(hash1).not.toBe(hash2); // Different salts
expect(service.verifyPassword(password, hash1)).toBe(true);
expect(service.verifyPassword("wrong", hash1)).toBe(false);

// Session management
const token = service.createSession("user-123");
expect(token.length).toBeGreaterThan(32);

const userId = service.verifySession(token);
expect(userId).toBe("user-123");

// Revoke session
service.revokeSession(token);
expect(service.verifySession(token)).toBeNull();
```

**Bilingual Output**: N/A (Backend service)

**Story Points**: 6

**Dependencies**: Task 1.3, 1.4

---

### Task 1.6: Create Audit Service

**Description**: Implement audit logging service to track authentication and profile events.

**File**: `backend/services/audit_service.ts`

**Acceptance Criteria**:
- [ ] `logSignup(userId, email, language)` - logs signup event
- [ ] `logSignin(userId)` - logs signin event
- [ ] `logProfileUpdate(userId, changedFields)` - logs what was updated
- [ ] `logContentAccess(userId, contentTier, allowed)` - logs content access attempts
- [ ] All logs include timestamp, user_id, action, resource, details
- [ ] Details stored as JSON string in audit_logs.details
- [ ] Optional IP address and User-Agent captured
- [ ] Query audit logs by userId, action, or date range
- [ ] No sensitive data (passwords, tokens) logged

**Test Case**:
```typescript
import { AuditService } from "../services/audit_service";

const service = new AuditService(db);

// Log signup
await service.logSignup("user-123", "test@example.com", "en");

// Query audit logs
const logs = await service.getLogsForUser("user-123");
expect(logs.length).toBeGreaterThan(0);
expect(logs[0].action).toBe("signup");
expect(logs[0].details).toContain("test@example.com");
```

**Bilingual Output**: N/A (Backend service)

**Story Points**: 4

**Dependencies**: Task 1.1, 1.2, 1.4

---

### Task 1.7: Create Signup Endpoint (REST API)

**Description**: Implement POST /api/auth/signup endpoint with full validation and profile creation.

**File**: `backend/api/routes/auth.py` (POST /api/auth/signup)

**Acceptance Criteria**:
- [ ] Endpoint accepts SignupRequest with email, password, name, profile, language_preference
- [ ] Validates email format (RFC 5322)
- [ ] Validates password strength (12+ chars)
- [ ] Checks email uniqueness (returns 409 Conflict if exists)
- [ ] Creates user + profile in single transaction
- [ ] Returns 200 OK with session_token, user_id, email, name
- [ ] Logs signup event to audit_logs
- [ ] Handles all error cases (duplicate email, invalid input, server error)
- [ ] Response includes appropriate error messages
- [ ] No sensitive data in response (password hash, tokens in cookie only)

**Request Schema**:
```typescript
interface SignupRequest {
  email: string;
  password: string;
  name: string;
  language_preference: "en" | "ur";
  profile: {
    programming_years: string;
    languages: string[];
    ai_ml_level: string;
    ros_experience: string;
    hardware_platforms: string[];
    humanoid_experience: string;
    simulation_tools: string[];
    gpu_access: boolean;
    hardware_specs: string;
    lab_equipment_access: string[];
  };
}
```

**Response Schema**:
```typescript
interface SignupResponse {
  user_id: string;
  email: string;
  name: string;
  session_token: string;
  email_verified: boolean;
}
```

**Test Case**:
```python
import httpx

client = httpx.Client(base_url="http://localhost:8000")

response = client.post("/api/auth/signup", json={
    "email": "new@example.com",
    "password": "secure_password_12chars",
    "name": "New User",
    "language_preference": "en",
    "profile": {
        "programming_years": "10+",
        "languages": ["Python"],
        "ai_ml_level": "advanced",
        # ... all other fields
    }
})

assert response.status_code == 200
data = response.json()
assert data["user_id"]
assert data["session_token"]
assert "password" not in str(data)

# Verify user created in DB
user = db.query(users).filter(users.email == "new@example.com").first()
assert user is not None
assert user.name == "New User"

# Verify profile created
profile = db.query(userProfiles).filter(userProfiles.user_id == user.id).first()
assert profile.ai_ml_level == "advanced"

# Verify audit log
logs = db.query(auditLogs).filter(auditLogs.user_id == user.id).all()
assert any(log.action == "signup" for log in logs)
```

**Bilingual Output**: N/A (API backend)

**Story Points**: 8

**Dependencies**: Task 1.4, 1.5, 1.6

---

### Task 1.8: Create Signin Endpoint

**Description**: Implement POST /api/auth/signin endpoint with email/password verification.

**File**: `backend/api/routes/auth.py` (POST /api/auth/signin)

**Acceptance Criteria**:
- [ ] Endpoint accepts email and password
- [ ] Queries user by email
- [ ] Verifies password against bcrypt hash
- [ ] Returns 401 Unauthorized if email not found or password incorrect
- [ ] Returns 200 OK with session_token and user data
- [ ] Includes user profile in response (for chat personalization)
- [ ] Logs signin event
- [ ] Rate limiting to prevent brute force (max 5 attempts per 15 min)
- [ ] No error message distinguishing "user not found" vs "wrong password"

**Test Case**:
```python
# Create user first
signup_response = client.post("/api/auth/signup", json={...})
user_id = signup_response.json()["user_id"]

# Signin with correct password
signin_response = client.post("/api/auth/signin", json={
    "email": "new@example.com",
    "password": "secure_password_12chars"
})

assert signin_response.status_code == 200
data = signin_response.json()
assert data["session_token"]
assert data["user_id"] == user_id

# Signin with wrong password
wrong_response = client.post("/api/auth/signin", json={
    "email": "new@example.com",
    "password": "wrong_password"
})

assert wrong_response.status_code == 401
assert "Invalid email or password" in wrong_response.json()["detail"]
```

**Bilingual Output**: N/A (API backend)

**Story Points**: 6

**Dependencies**: Task 1.4, 1.5, 1.6

---

### Task 1.9: Create Session Verification Endpoint

**Description**: Implement GET /api/auth/session to verify and return current user session.

**File**: `backend/api/routes/auth.py` (GET /api/auth/session)

**Acceptance Criteria**:
- [ ] Requires valid session token in Authorization header (Bearer token)
- [ ] Returns 401 Unauthorized if token missing or invalid
- [ ] Returns user object with id, email, name
- [ ] Returns user profile with all background fields
- [ ] Includes language_preference in response
- [ ] Response is fast (< 100ms including profile load)
- [ ] Profile is cached by session (5-minute TTL)

**Test Case**:
```python
# Get session with valid token
session_response = client.get(
    "/api/auth/session",
    headers={"Authorization": f"Bearer {session_token}"}
)

assert session_response.status_code == 200
data = session_response.json()
assert data["user"]["id"]
assert data["profile"]["ai_ml_level"]

# Without token
no_token = client.get("/api/auth/session")
assert no_token.status_code == 401
```

**Bilingual Output**: N/A (API backend)

**Story Points**: 4

**Dependencies**: Task 1.5

---

### Task 1.10: Create Signout Endpoint

**Description**: Implement POST /api/auth/signout to revoke session.

**File**: `backend/api/routes/auth.py` (POST /api/auth/signout)

**Acceptance Criteria**:
- [ ] Requires valid session token
- [ ] Deletes session from database
- [ ] Returns 200 OK
- [ ] Subsequent requests with revoked token return 401
- [ ] Logs signout event
- [ ] Frontend can clear localStorage after 200 response

**Test Case**:
```python
# Signout with valid token
signout = client.post(
    "/api/auth/signout",
    headers={"Authorization": f"Bearer {session_token}"}
)

assert signout.status_code == 200

# Token now invalid
session = client.get(
    "/api/auth/session",
    headers={"Authorization": f"Bearer {session_token}"}
)
assert session.status_code == 401
```

**Bilingual Output**: N/A (API backend)

**Story Points**: 3

**Dependencies**: Task 1.5

---

### Task 1.11: Create Content Gating Middleware

**Description**: Implement middleware to enforce content access based on user AI/ML level.

**File**: `backend/api/middleware/gating.py`

**Acceptance Criteria**:
- [ ] Middleware runs on protected routes (/content/*, /capstone/*, etc.)
- [ ] Loads user profile from session context
- [ ] Compares ai_ml_level against required tier
- [ ] Returns 403 Forbidden with error message in user's language if access denied
- [ ] Error response includes current_level, required_level, upgrade_link
- [ ] Beginner can only access ["basics", "fundamentals"]
- [ ] Intermediate can access ["basics", "fundamentals", "intermediate", "projects"]
- [ ] Advanced can access all content
- [ ] Performance: < 50ms overhead per request

**Test Case**:
```python
# Create beginner user
beginner_profile = {..., "ai_ml_level": "beginner"}

# Try to access capstone (requires advanced)
response = client.get(
    "/content/capstone/humanoid-control",
    headers={"Authorization": f"Bearer {beginner_token}"}
)

assert response.status_code == 403
error = response.json()
assert "Access denied" in error["error"]
assert error["current_level"] == "beginner"
assert error["required_level"] == "advanced"
```

**Bilingual Output**: Error messages in user's language_preference

**Story Points**: 6

**Dependencies**: Task 1.4, 1.5, 1.9

---

### Task 1.12: Create Profile Update Endpoint

**Description**: Implement PATCH /api/auth/profile to allow users to update background info.

**File**: `backend/api/routes/auth.py` (PATCH /api/auth/profile)

**Acceptance Criteria**:
- [ ] Requires authentication
- [ ] Accepts partial profile updates (not all fields required)
- [ ] Validates field values (enums, arrays, booleans)
- [ ] Updates user_profiles table
- [ ] Logs profile_update event with changed fields
- [ ] Returns updated profile
- [ ] Invalidates profile cache so next session sees new data
- [ ] Triggers chat system refresh (notification to frontend)

**Test Case**:
```python
# Update AI/ML level
update = client.patch(
    "/api/auth/profile",
    json={"ai_ml_level": "intermediate"},
    headers={"Authorization": f"Bearer {token}"}
)

assert update.status_code == 200
updated = update.json()
assert updated["ai_ml_level"] == "intermediate"

# Verify audit log
logs = db.query(auditLogs).filter(auditLogs.user_id == user_id).all()
update_log = next((l for l in logs if l.action == "profile_update"), None)
assert update_log is not None
```

**Bilingual Output**: N/A (API backend)

**Story Points**: 5

**Dependencies**: Task 1.4, 1.5, 1.9

---

### Task 1.13: Setup Email Verification (Optional)

**Description**: Configure email verification flow for optional email confirmation during signup.

**File**: `backend/services/email_service.ts`

**Acceptance Criteria**:
- [ ] Email verification optional (controlled by ENABLE_EMAIL_VERIFICATION env var)
- [ ] Create email_verification_tokens table with token, user_id, expires_at
- [ ] `sendVerificationEmail(email)` generates token and sends email
- [ ] Email includes verification link: `{FRONTEND_URL}/verify?token={token}`
- [ ] Verification link valid for 24 hours
- [ ] GET /api/auth/verify?token=... verifies and sets user.email_verified = true
- [ ] Error if token expired or invalid
- [ ] No verification required for login (email_verified = false is okay)

**Test Case** (only if ENABLE_EMAIL_VERIFICATION=true):
```python
# Check env var
import os
if os.getenv("ENABLE_EMAIL_VERIFICATION") != "true":
    pytest.skip("Email verification disabled")

# Signup sends verification email
signup = client.post("/api/auth/signup", json={...})
assert signup.status_code == 200

# Email sent (would be checked by email service mock)
# Token generated in DB
token = db.query(emailVerificationTokens).filter(
    emailVerificationTokens.user_id == user_id
).first()
assert token is not None

# Verify email
verify = client.get(f"/api/auth/verify?token={token.token}")
assert verify.status_code == 200

user = db.query(users).filter(users.id == user_id).first()
assert user.email_verified == True
```

**Bilingual Output**: Email templates in English (Urdu optional for Phase 2+)

**Story Points**: 5

**Dependencies**: Task 1.4, 1.5

---

## Phase 2: Frontend - i18n Setup & Translation Files (8 tasks, ~20 SP)

### Task 2.1: Initialize i18n Configuration

**Description**: Set up react-i18next with i18next-http-backend for loading translations.

**File**: `website/src/lib/i18n/config.ts`

**Acceptance Criteria**:
- [ ] i18n initialized with fallbackLng: "en"
- [ ] Namespaces defined: ["common", "auth", "forms", "errors", "chat"]
- [ ] Backend loader configured with loadPath: "/locales/{{lng}}/{{ns}}.json"
- [ ] React Suspense disabled (useSuspense: false)
- [ ] Language detection based on browser locale first, then localStorage
- [ ] Default language in localStorage if no browser locale match
- [ ] Exported for use in components

**Test Case**:
```typescript
import i18n from "../lib/i18n/config";

expect(i18n).toBeDefined();
expect(i18n.defaultNS).toBe("common");
expect(i18n.ns).toContain("auth");
expect(i18n.ns).toContain("forms");

// Load English
await i18n.changeLanguage("en");
expect(i18n.language).toBe("en");

// Load Urdu
await i18n.changeLanguage("ur");
expect(i18n.language).toBe("ur");
```

**Bilingual Output**: N/A (Configuration)

**Story Points**: 3

**Dependencies**: None

---

### Task 2.2: Create English Translation Files - Auth

**Description**: Create English translations for all authentication flows.

**File**: `website/public/locales/en/auth.json`

**Acceptance Criteria**:
- [ ] File contains all keys for signup flow: step_indicator, email_label, password_label, etc.
- [ ] All form labels (12+ keys)
- [ ] All button labels (Next, Back, Complete, Sign In, Sign Out)
- [ ] All error messages (13+ keys)
- [ ] All validation messages (invalid email, password mismatch, etc.)
- [ ] Interpolation placeholders for dynamic content ({{current}}, {{total}})
- [ ] No hardcoded text outside this file
- [ ] Valid JSON format (tested with JSON parser)

**Keys Required** (minimum):
- `signup_title`, `signup_subtitle`
- `signin_title`, `signin_subtitle`
- `signout_title`
- `step_indicator`
- `email_label`, `email_placeholder`
- `password_label`, `password_placeholder`
- `confirm_password_label`, `confirm_password_placeholder`
- `name_label`, `name_placeholder`
- `next_button`, `back_button`, `complete_button`, `signin_button`, `signout_button`
- `error_email_invalid`, `error_email_exists`, `error_password_short`, `error_password_mismatch`
- `error_name_required`, `error_invalid_request`, `error_server_error`
- `validation_email_invalid`, `validation_password_weak`
- `already_have_account`, `dont_have_account`
- `forgot_password`
- `email_verification_sent`, `verification_pending`

**Test Case**:
```typescript
import authEn from "../../../public/locales/en/auth.json";

// All required keys present
const requiredKeys = [
  "signup_title", "email_label", "next_button",
  "error_email_exists", "validation_email_invalid"
];

for (const key of requiredKeys) {
  expect(authEn[key]).toBeDefined();
  expect(typeof authEn[key]).toBe("string");
}

// No empty strings
Object.values(authEn).forEach(value => {
  expect(value.trim().length).toBeGreaterThan(0);
});

// Valid JSON
expect(() => JSON.stringify(authEn)).not.toThrow();
```

**Bilingual Output**: English translations only; Urdu in Task 2.3

**Story Points**: 4

**Dependencies**: Task 2.1

---

### Task 2.3: Create Urdu Translation Files - Auth

**Description**: Create Urdu translations for all authentication flows (mirror of Task 2.2).

**File**: `website/public/locales/ur/auth.json`

**Acceptance Criteria**:
- [ ] All keys match English file exactly
- [ ] Urdu translations are accurate and natural (not transliteration)
- [ ] Technical terms preserved in English where appropriate (e.g., "email", "password")
- [ ] Grammatical gender/formality appropriate (formal register for interface)
- [ ] No hardcoded Urdu text in components
- [ ] Valid JSON format

**Urdu Translations** (examples):
```json
{
  "signup_title": "اپنا اکاؤنٹ بنائیں",
  "signup_subtitle": "شخصی سیکھنے کے لیے Physical AI کمیونٹی میں شامل ہوں",
  "email_label": "ای میل پتہ",
  "password_label": "پاس ورڈ",
  "next_button": "اگلا",
  "back_button": "واپس",
  "complete_button": "اکاؤنٹ بنائیں",
  "error_email_exists": "یہ ای میل پہلے سے رجسٹرڈ ہے",
  "error_password_mismatch": "پاس ورڈ مماثل نہیں ہیں"
}
```

**Test Case**:
```typescript
import authEn from "../../../public/locales/en/auth.json";
import authUr from "../../../public/locales/ur/auth.json";

// All English keys exist in Urdu
const enKeys = Object.keys(authEn);
const urKeys = Object.keys(authUr);

for (const key of enKeys) {
  expect(urKeys).toContain(key);
}

// No English words in Urdu (except technical terms)
const urText = Object.values(authUr).join(" ");
// Check for common words accidentally left in English
const commonEnWords = ["email", "password"]; // These are OK
const forbiddenWords = ["the", "and", "or"];
for (const word of forbiddenWords) {
  expect(urText.toLowerCase()).not.toContain(` ${word} `);
}
```

**Bilingual Output**: Urdu translations

**Story Points**: 4

**Dependencies**: Task 2.2

---

### Task 2.4: Create English Translation Files - Forms

**Description**: Create English translations for all background question forms.

**File**: `website/public/locales/en/forms.json`

**Acceptance Criteria**:
- [ ] All form section titles (2 sections: Software, Hardware & Robotics)
- [ ] All question labels (programming_years_label, languages_label, etc.)
- [ ] All radio/checkbox option labels (e.g., "Beginner - New to programming/robotics")
- [ ] All help text (e.g., "Select all that apply")
- [ ] Dropdown options for enums (0, 1-3, 3-5, 5-10, 10+)
- [ ] AI/ML levels (None, Beginner, Intermediate, Advanced)
- [ ] ROS experience levels (None, Basic, Intermediate, Advanced)
- [ ] Humanoid experience levels (None, Simulation only, Physical hardware, Research level)
- [ ] Hardware specs (Basic, Standard, High-performance)
- [ ] Valid JSON, no empty strings

**Keys Required** (minimum):
- `software_background_title`
- `programming_years_label`, `programming_years_help`
- `programming_years_0`, `programming_years_1-3`, `programming_years_3-5`, `programming_years_5-10`, `programming_years_10+`
- `languages_label`, `languages_help`
- `languages_python`, `languages_javascript`, `languages_typescript`, `languages_c++`, `languages_java`, `languages_c#`, `languages_go`, `languages_rust`
- `ai_ml_level_label`
- `ai_ml_none`, `ai_ml_beginner`, `ai_ml_intermediate`, `ai_ml_advanced`
- `robotics_background_title`
- `ros_experience_label`
- `ros_none`, `ros_basic`, `ros_intermediate`, `ros_advanced`
- `hardware_platforms_label`, `hardware_platforms_help`
- `hardware_arduino`, `hardware_raspberry_pi`, `hardware_jetson`, `hardware_microcontrollers`, `hardware_sensors`, `hardware_robot_kits`
- `humanoid_experience_label`
- `humanoid_none`, `humanoid_simulation`, `humanoid_physical`, `humanoid_research`
- `simulation_tools_label`, `simulation_tools_help`
- `simulation_gazebo`, `simulation_isaac_sim`, `simulation_coppeliasim`
- `gpu_access_label`, `cpu_ram_label`, `lab_equipment_label`
- `hardware_specs_basic`, `hardware_specs_standard`, `hardware_specs_high_performance`

**Test Case**:
```typescript
import formsEn from "../../../public/locales/en/forms.json";

// All enum options present
const programmingYears = ["0", "1-3", "3-5", "5-10", "10+"];
for (const year of programmingYears) {
  expect(formsEn[`programming_years_${year}`]).toBeDefined();
}

// All AI/ML levels present
const aiMlLevels = ["none", "beginner", "intermediate", "advanced"];
for (const level of aiMlLevels) {
  expect(formsEn[`ai_ml_${level}`]).toBeDefined();
}
```

**Bilingual Output**: English only; Urdu in Task 2.5

**Story Points**: 5

**Dependencies**: Task 2.1

---

### Task 2.5: Create Urdu Translation Files - Forms

**Description**: Create Urdu translations for all form labels and options.

**File**: `website/public/locales/ur/forms.json`

**Acceptance Criteria**:
- [ ] All keys match English exactly
- [ ] Urdu translations accurate and natural
- [ ] Enums translated clearly (e.g., "10+ سال" instead of just "10+")
- [ ] Levels translated with clear hierarchy (ابتدائی, درمیانی, ماہر)
- [ ] Hardware names partially transliterated where needed (Arduino, ROS, Gazebo) but with Urdu prefix/suffix

**Urdu Translations** (examples):
```json
{
  "software_background_title": "آپ کا پروگرامنگ کا پس منظر",
  "programming_years_label": "پروگرامنگ میں سال کی تجربہ",
  "programming_years_10+": "10 سے زیادہ سال",
  "languages_label": "اپنی پروگرامنگ زبانیں منتخب کریں",
  "languages_python": "Python",
  "ai_ml_level_label": "اپنی AI/ML کی مہارت کی سطح منتخب کریں",
  "ai_ml_beginner": "ابتدائی",
  "ai_ml_intermediate": "درمیانی",
  "ai_ml_advanced": "ماہر",
  "robotics_background_title": "رابوٹکس اور ہارڈویئر کا پس منظر",
  "ros_experience_label": "ROS کا کتنا تجربہ ہے؟",
  "ros_none": "کوئی نہیں",
  "humanoid_experience_label": "ہیومینوئڈ روبوٹ کے ساتھ کتنا کام کیا؟",
  "humanoid_simulation": "صرف سمیولیشن",
  "humanoid_physical": "فزیکل ہارڈویئر",
  "humanoid_research": "ریسرچ لیول"
}
```

**Test Case**:
```typescript
import formsEn from "../../../public/locales/en/forms.json";
import formsUr from "../../../public/locales/ur/forms.json";

// Same keys
expect(Object.keys(formsUr)).toEqual(Object.keys(formsEn));

// Sample key has Urdu text
expect(formsUr["ai_ml_beginner"]).toMatch(/ابتدائی|شروع|نیا/);
```

**Bilingual Output**: Urdu translations

**Story Points**: 5

**Dependencies**: Task 2.4

---

### Task 2.6: Create Error & Common Translation Files

**Description**: Create translation files for error messages and common UI elements.

**Files**:
- `website/public/locales/en/errors.json`
- `website/public/locales/ur/errors.json`

**Acceptance Criteria**:
- [ ] All error scenarios covered:
  - `error_invalid_email`
  - `error_email_exists`
  - `error_password_too_short`
  - `error_password_mismatch`
  - `error_server_error`
  - `error_network_error`
  - `error_session_expired`
  - `error_access_denied`
  - `error_not_authenticated`
  - `error_profile_incomplete`
- [ ] All common elements:
  - `loading`, `submitting`, `error`, `success`
  - `required_field`
  - `loading_page`, `loading_translations`
- [ ] English and Urdu versions match exactly in keys

**Test Case**:
```typescript
import errorsEn from "../../../public/locales/en/errors.json";
import errorsUr from "../../../public/locales/ur/errors.json";

// Verify key parity
expect(Object.keys(errorsEn).sort()).toEqual(Object.keys(errorsUr).sort());

// Verify all critical errors present
const criticalErrors = [
  "error_email_exists",
  "error_password_mismatch",
  "error_access_denied"
];
for (const err of criticalErrors) {
  expect(errorsEn[err]).toBeDefined();
  expect(errorsUr[err]).toBeDefined();
}
```

**Bilingual Output**: English and Urdu error messages

**Story Points**: 4

**Dependencies**: Task 2.1

---

### Task 2.7: Create Chat Translation Files

**Description**: Create translation files for RAG chat interface.

**Files**:
- `website/public/locales/en/chat.json`
- `website/public/locales/ur/chat.json`

**Acceptance Criteria**:
- [ ] Chat labels: `chat_title`, `ask_question`, `send_button`, `clear_history`
- [ ] Chat messages: `thinking`, `response_ready`, `error_loading`
- [ ] Personalization messages: `filtered_by_level`, `advanced_only`, `beginner_friendly`
- [ ] Citations: `source`, `sources`, `chapter`, `section`
- [ ] Sign in prompt: `signin_to_chat`, `signin_required`
- [ ] English and Urdu versions with matching keys

**Test Case**:
```typescript
import chatEn from "../../../public/locales/en/chat.json";
import chatUr from "../../../public/locales/ur/chat.json";

expect(Object.keys(chatEn)).toEqual(Object.keys(chatUr));
expect(chatEn["chat_title"]).toBeDefined();
expect(chatUr["signin_required"]).toBeDefined();
```

**Bilingual Output**: English and Urdu chat messages

**Story Points**: 3

**Dependencies**: Task 2.1

---

### Task 2.8: Validate Translation Completeness

**Description**: Create validation script to ensure all translation files are complete and consistent.

**File**: `website/scripts/validate-translations.ts`

**Acceptance Criteria**:
- [ ] Script checks all namespaces have matching English/Urdu keys
- [ ] Detects missing keys (error if English has key but Urdu doesn't)
- [ ] Detects unused keys (warning if key not used in components)
- [ ] Validates JSON syntax
- [ ] Generates report of translation coverage (target: 100%)
- [ ] Can be run in CI/CD pipeline
- [ ] Executable: `npm run validate-translations`

**Test Case**:
```bash
# Run validation
npm run validate-translations

# Expected output:
# ✓ auth.json: 25/25 keys (100%)
# ✓ forms.json: 40/40 keys (100%)
# ✓ errors.json: 15/15 keys (100%)
# ✓ chat.json: 12/12 keys (100%)
# All namespaces complete!
```

**Bilingual Output**: Validation report with English summary

**Story Points**: 3

**Dependencies**: Task 2.2 - 2.7

---

## Phase 3: Frontend - Components & UI (12 tasks, ~45 SP)

### Task 3.1: Create Language Selector Component

**Description**: Implement language toggle in header for English/Urdu switching.

**File**: `website/src/components/LanguageSelector.tsx`

**Acceptance Criteria**:
- [ ] Component renders 2 buttons: "English" and "اردو"
- [ ] Clicking button changes language via i18n.changeLanguage()
- [ ] Selected language button visually highlighted (active class)
- [ ] Selection saved to localStorage ("language_preference")
- [ ] Selection persists across page reloads
- [ ] Re-render time < 300ms (measured with React DevTools Profiler)
- [ ] Accessible with keyboard (Tab to focus, Space/Enter to select)
- [ ] Mobile-responsive (buttons stack on small screens)

**Code Outline**:
```typescript
export const LanguageSelector: React.FC = () => {
  const { i18n } = useTranslation();

  const handleLanguageChange = (lang: "en" | "ur") => {
    i18n.changeLanguage(lang);
    localStorage.setItem("language_preference", lang);
  };

  return (
    <div className={styles.languageSelector}>
      <button
        onClick={() => handleLanguageChange("en")}
        className={i18n.language === "en" ? styles.active : ""}
        aria-label="English"
      >
        English
      </button>
      <button
        onClick={() => handleLanguageChange("ur")}
        className={i18n.language === "ur" ? styles.active : ""}
        aria-label="اردو"
      >
        اردو
      </button>
    </div>
  );
};
```

**Test Case**:
```typescript
import { render, screen, fireEvent } from "@testing-library/react";
import { LanguageSelector } from "./LanguageSelector";

test("renders language selector", () => {
  render(<LanguageSelector />);

  expect(screen.getByText("English")).toBeInTheDocument();
  expect(screen.getByText("اردو")).toBeInTheDocument();
});

test("switches language on click", async () => {
  render(<LanguageSelector />);

  const urduButton = screen.getByText("اردو");
  fireEvent.click(urduButton);

  // Verify i18n changed
  await waitFor(() => {
    expect(localStorage.getItem("language_preference")).toBe("ur");
  });
});
```

**Bilingual Output**: Button labels in English and Urdu

**Story Points**: 5

**Dependencies**: Task 2.1

---

### Task 3.2: Create MultiStepSignup Form Component (Controller)

**Description**: Main signup form controller managing 4-step flow, validation, and state.

**File**: `website/src/components/auth/MultiStepSignupForm.tsx`

**Acceptance Criteria**:
- [ ] Manages currentStep state (1-4)
- [ ] Renders progress indicator showing "Step X of 4"
- [ ] Progress bar fills as user advances
- [ ] Back button disabled on Step 1
- [ ] Submit button only on Step 4 (labeled "Create Account")
- [ ] Validates each step before advancing
- [ ] Stores form data in component state
- [ ] On completion, calls API and handles response
- [ ] Shows loading spinner during submission
- [ ] Shows success message and redirects to dashboard
- [ ] Shows error messages (in selected language)
- [ ] Responsive layout (full width on mobile, centered on desktop)

**Props**:
```typescript
interface MultiStepSignupFormProps {
  onSuccess?: (userId: string) => void;
  redirectUrl?: string;
}
```

**Code Outline**:
```typescript
export const MultiStepSignupForm: React.FC<MultiStepSignupFormProps> = ({
  onSuccess,
  redirectUrl = "/dashboard"
}) => {
  const { t } = useTranslation(["auth", "forms"]);
  const [currentStep, setCurrentStep] = useState(1);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const navigate = useNavigate();

  const {
    control,
    watch,
    handleSubmit,
    trigger,
    formState: { errors, isValid }
  } = useForm({ defaultValues: {...} });

  const onSubmit = async (data) => {
    try {
      setIsSubmitting(true);
      const response = await apiClient.post("/api/auth/signup", data);
      localStorage.setItem("session_token", response.data.session_token);
      setTimeout(() => navigate(redirectUrl), 500);
    } catch (err) {
      setError(err.message);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleNext = async () => {
    const isStepValid = await trigger(); // Validate current step
    if (isStepValid && currentStep < 4) {
      setCurrentStep(currentStep + 1);
    }
  };

  const handleBack = () => {
    if (currentStep > 1) setCurrentStep(currentStep - 1);
  };

  return (
    <form onSubmit={handleSubmit(onSubmit)}>
      {/* Progress */}
      <div className={styles.progressIndicator}>
        <div className={styles.stepLabel}>
          {t("auth:step_indicator", { current: currentStep, total: 4 })}
        </div>
        {/* Step rendering logic */}
      </div>

      {/* Error display */}
      {error && <div className={styles.error}>{error}</div>}

      {/* Navigation */}
      <div className={styles.buttons}>
        {currentStep > 1 && (
          <button type="button" onClick={handleBack}>
            {t("auth:back_button")}
          </button>
        )}
        {currentStep < 4 && (
          <button type="button" onClick={handleNext}>
            {t("auth:next_button")}
          </button>
        )}
        {currentStep === 4 && (
          <button type="submit" disabled={isSubmitting}>
            {isSubmitting ? t("auth:submitting") : t("auth:complete_button")}
          </button>
        )}
      </div>
    </form>
  );
};
```

**Test Case**:
```typescript
test("completes 4-step signup", async () => {
  const mockNavigate = jest.fn();
  render(
    <MultiStepSignupForm onSuccess={mockNavigate} />
  );

  // Step 1: Fill credentials
  fireEvent.change(screen.getByLabelText("Email"), { target: { value: "test@example.com" } });
  fireEvent.change(screen.getByLabelText("Password"), { target: { value: "secure_12chars" } });
  fireEvent.click(screen.getByText("Next"));

  // Step 2, 3, 4... (similar)

  // Final submit
  fireEvent.click(screen.getByText("Create Account"));

  // Verify success
  await waitFor(() => {
    expect(mockNavigate).toHaveBeenCalledWith("/dashboard");
  });
});
```

**Bilingual Output**: All labels and buttons in selected language

**Story Points**: 10

**Dependencies**: Task 2.1, 2.2, 2.3

---

### Task 3.3: Create SignupStep1 Component (Credentials)

**Description**: Step 1 form fields - Email, Password, Confirm Password, Full Name.

**File**: `website/src/components/auth/SignupStep1.tsx`

**Acceptance Criteria**:
- [ ] Text input for email with validation (format, uniqueness check on blur)
- [ ] Password input with strength indicator (min 12 chars)
- [ ] Confirm password input with match validation
- [ ] Text input for full name (required)
- [ ] All labels from i18n ("auth:email_label", etc.)
- [ ] Error messages from i18n ("auth:error_email_invalid", etc.)
- [ ] Real-time validation with visual feedback
- [ ] Email field shows "email already registered" if exists (API check)
- [ ] Password field shows strength meter (Weak/Fair/Good/Strong)
- [ ] Accessible with proper ARIA labels

**Code Outline**:
```typescript
interface SignupStep1Props {
  control: Control<any>;
  errors: FieldErrors<any>;
}

export const SignupStep1: React.FC<SignupStep1Props> = ({ control, errors }) => {
  const { t } = useTranslation("auth");

  return (
    <div className={styles.step}>
      <div className={styles.formGroup}>
        <label>{t("email_label")} *</label>
        <Controller
          name="email"
          control={control}
          rules={{
            required: t("required_field"),
            pattern: {
              value: /^[^@]+@[^@]+\.[^@]+$/,
              message: t("error_email_invalid")
            },
            validate: async (value) => {
              // API check for email uniqueness
              const exists = await checkEmailExists(value);
              return !exists || t("error_email_exists");
            }
          }}
          render={({ field }) => (
            <>
              <input
                {...field}
                type="email"
                placeholder={t("email_placeholder")}
                aria-label={t("email_label")}
              />
              {errors.email && <span className={styles.error}>{errors.email.message}</span>}
            </>
          )}
        />
      </div>

      <div className={styles.formGroup}>
        <label>{t("password_label")} *</label>
        <Controller
          name="password"
          control={control}
          rules={{
            required: t("required_field"),
            minLength: {
              value: 12,
              message: t("error_password_too_short")
            }
          }}
          render={({ field }) => (
            <>
              <input
                {...field}
                type="password"
                placeholder={t("password_placeholder")}
              />
              <PasswordStrengthMeter password={field.value} />
              {errors.password && <span className={styles.error}>{errors.password.message}</span>}
            </>
          )}
        />
      </div>

      {/* Confirm password, name... similar */}
    </div>
  );
};
```

**Test Case**:
```typescript
test("validates email format", async () => {
  render(<SignupStep1 control={control} errors={errors} />);

  const emailInput = screen.getByLabelText("Email");
  fireEvent.change(emailInput, { target: { value: "invalid-email" } });

  await waitFor(() => {
    expect(screen.getByText("Email is invalid")).toBeInTheDocument();
  });
});

test("checks email uniqueness", async () => {
  render(<SignupStep1 control={control} errors={errors} />);

  const emailInput = screen.getByLabelText("Email");
  fireEvent.change(emailInput, { target: { value: "existing@example.com" } });
  fireEvent.blur(emailInput);

  await waitFor(() => {
    expect(screen.getByText("This email is already registered")).toBeInTheDocument();
  });
});
```

**Bilingual Output**: Labels, placeholders, and error messages in selected language

**Story Points**: 8

**Dependencies**: Task 2.2, 2.3, 2.6

---

### Task 3.4: Create SignupStep2 Component (Software Background)

**Description**: Step 2 form fields - Programming experience, languages, AI/ML level.

**File**: `website/src/components/auth/SignupStep2.tsx`

**Acceptance Criteria**:
- [ ] Dropdown for programming_years (0, 1-3, 3-5, 5-10, 10+)
- [ ] Multi-select checkbox group for languages (Python, JavaScript, C++, etc.)
- [ ] Radio button group for ai_ml_level (Beginner, Intermediate, Advanced, None)
- [ ] All options from "forms" translation namespace
- [ ] At least one language required validation
- [ ] Help text under multi-select ("Select all that apply")
- [ ] Accessible with ARIA labels and descriptions

**Code Outline**:
```typescript
export const SignupStep2: React.FC<SignupStep2Props> = ({ control, errors }) => {
  const { t } = useTranslation(["forms", "auth"]);

  const LANGUAGE_OPTIONS = [
    "python", "javascript", "typescript", "c++", "java", "c#", "go", "rust"
  ];

  const AI_ML_OPTIONS = ["none", "beginner", "intermediate", "advanced"];

  return (
    <div className={styles.step}>
      <h2>{t("forms:software_background_title")}</h2>

      {/* Programming years */}
      <div className={styles.formGroup}>
        <label>{t("forms:programming_years_label")} *</label>
        <Controller
          name="programming_years"
          control={control}
          rules={{ required: t("auth:required_field") }}
          render={({ field }) => (
            <select {...field} aria-label={t("forms:programming_years_label")}>
              <option value="">Select...</option>
              {["0", "1-3", "3-5", "5-10", "10+"].map(year => (
                <option key={year} value={year}>
                  {t(`forms:programming_years_${year}`)}
                </option>
              ))}
            </select>
          )}
        />
      </div>

      {/* Languages */}
      <div className={styles.formGroup}>
        <label>{t("forms:languages_label")} *</label>
        <p className={styles.helpText}>{t("forms:languages_help")}</p>
        <Controller
          name="languages"
          control={control}
          rules={{
            required: t("auth:required_field"),
            validate: (value) => value.length > 0 || "Select at least one language"
          }}
          render={({ field: { value, onChange } }) => (
            <div className={styles.checkboxGroup}>
              {LANGUAGE_OPTIONS.map(lang => (
                <label key={lang} className={styles.checkbox}>
                  <input
                    type="checkbox"
                    checked={value.includes(lang)}
                    onChange={(e) => {
                      if (e.target.checked) {
                        onChange([...value, lang]);
                      } else {
                        onChange(value.filter((l) => l !== lang));
                      }
                    }}
                  />
                  {t(`forms:languages_${lang.toLowerCase()}`)}
                </label>
              ))}
            </div>
          )}
        />
      </div>

      {/* AI/ML Level */}
      <div className={styles.formGroup}>
        <label>{t("forms:ai_ml_level_label")} *</label>
        <Controller
          name="ai_ml_level"
          control={control}
          rules={{ required: t("auth:required_field") }}
          render={({ field }) => (
            <div className={styles.radioGroup}>
              {AI_ML_OPTIONS.map(option => (
                <label key={option} className={styles.radio}>
                  <input
                    type="radio"
                    {...field}
                    value={option}
                    checked={field.value === option}
                  />
                  {t(`forms:ai_ml_${option}`)}
                </label>
              ))}
            </div>
          )}
        />
      </div>
    </div>
  );
};
```

**Test Case**:
```typescript
test("validates at least one language selected", async () => {
  render(<SignupStep2 control={control} errors={errors} />);

  // Try to submit without selecting language
  expect(errors.languages?.message).toBe("Select at least one language");
});

test("displays translations in Urdu", async () => {
  // Mock i18n to Urdu
  i18n.changeLanguage("ur");
  render(<SignupStep2 control={control} errors={errors} />);

  // Verify Urdu labels
  expect(screen.getByText("آپ کی پروگرامنگ کی زبانیں")).toBeInTheDocument();
});
```

**Bilingual Output**: All labels, options, and help text in selected language

**Story Points**: 8

**Dependencies**: Task 2.4, 2.5

---

### Task 3.5: Create SignupStep3 Component (Robotics Background)

**Description**: Step 3 form fields - ROS experience, hardware platforms, humanoid experience, simulation tools.

**File**: `website/src/components/auth/SignupStep3.tsx`

**Acceptance Criteria**:
- [ ] Radio button group for ros_experience (None, Basic, Intermediate, Advanced)
- [ ] Multi-select checkboxes for hardware_platforms (Arduino, RPi, Jetson, Microcontrollers, Sensors, Robot Kits)
- [ ] Radio button group for humanoid_experience (None, Simulation, Physical, Research)
- [ ] Multi-select checkboxes for simulation_tools (Gazebo, Isaac Sim, CoppeliaSim)
- [ ] All labels and options from "forms" translations
- [ ] Optional fields (no validation required)
- [ ] Accessible with proper ARIA labels

**Code Outline** (similar to Step 2 with different fields):
```typescript
export const SignupStep3: React.FC<SignupStep3Props> = ({ control, errors }) => {
  const { t } = useTranslation(["forms", "auth"]);

  return (
    <div className={styles.step}>
      <h2>{t("forms:robotics_background_title")}</h2>

      {/* ROS Experience */}
      <div className={styles.formGroup}>
        <label>{t("forms:ros_experience_label")}</label>
        <Controller
          name="ros_experience"
          control={control}
          render={({ field }) => (
            <div className={styles.radioGroup}>
              {["none", "basic", "intermediate", "advanced"].map(option => (
                <label key={option} className={styles.radio}>
                  <input type="radio" {...field} value={option} />
                  {t(`forms:ros_${option}`)}
                </label>
              ))}
            </div>
          )}
        />
      </div>

      {/* Hardware Platforms */}
      <div className={styles.formGroup}>
        <label>{t("forms:hardware_platforms_label")}</label>
        <p className={styles.helpText}>{t("forms:hardware_platforms_help")}</p>
        {/* Multi-select checkboxes */}
      </div>

      {/* Humanoid Experience, Simulation Tools... */}
    </div>
  );
};
```

**Test Case**:
```typescript
test("allows optional robotics fields", () => {
  render(<SignupStep3 control={control} errors={errors} />);

  // No errors when fields not filled
  expect(errors.ros_experience).toBeUndefined();
  expect(errors.hardware_platforms).toBeUndefined();
});
```

**Bilingual Output**: All labels and options in selected language

**Story Points**: 8

**Dependencies**: Task 2.4, 2.5

---

### Task 3.6: Create SignupStep4 Component (Hardware Access)

**Description**: Step 4 form fields - GPU access, hardware specs, lab equipment.

**File**: `website/src/components/auth/SignupStep4.tsx`

**Acceptance Criteria**:
- [ ] Checkbox for gpu_access (Yes/No)
- [ ] Dropdown for hardware_specs (Basic, Standard, High-performance)
- [ ] Checkbox group for lab_equipment_access (Motion capture, Force feedback, Sensors)
- [ ] Summary of profile before final submission
- [ ] All fields optional
- [ ] Labels and options from "forms" translations

**Code Outline**:
```typescript
export const SignupStep4: React.FC<SignupStep4Props> = ({ control, errors, allData }) => {
  const { t } = useTranslation(["forms", "auth"]);

  return (
    <div className={styles.step}>
      <h2>{t("forms:hardware_access_title")}</h2>

      {/* GPU Access */}
      <div className={styles.formGroup}>
        <Controller
          name="gpu_access"
          control={control}
          render={({ field }) => (
            <label className={styles.checkbox}>
              <input type="checkbox" {...field} />
              {t("forms:gpu_access_label")}
            </label>
          )}
        />
      </div>

      {/* Hardware Specs */}
      <div className={styles.formGroup}>
        <label>{t("forms:cpu_ram_label")}</label>
        <Controller
          name="hardware_specs"
          control={control}
          render={({ field }) => (
            <select {...field}>
              <option value="basic">{t("forms:hardware_specs_basic")}</option>
              <option value="standard">{t("forms:hardware_specs_standard")}</option>
              <option value="high-performance">{t("forms:hardware_specs_high_performance")}</option>
            </select>
          )}
        />
      </div>

      {/* Lab Equipment, Summary... */}
    </div>
  );
};
```

**Test Case**:
```typescript
test("displays summary of user profile", () => {
  const allData = {
    email: "test@example.com",
    name: "Test User",
    ai_ml_level: "advanced",
    hardware_platforms: ["Jetson"]
  };

  render(<SignupStep4 control={control} errors={errors} allData={allData} />);

  // Summary should show key info
  expect(screen.getByText("Test User")).toBeInTheDocument();
});
```

**Bilingual Output**: Labels and options in selected language

**Story Points**: 6

**Dependencies**: Task 2.4, 2.5

---

### Task 3.7: Create SigninForm Component

**Description**: Simple email/password signin form with bilingual support.

**File**: `website/src/components/auth/SigninForm.tsx`

**Acceptance Criteria**:
- [ ] Email input with validation
- [ ] Password input
- [ ] "Sign In" button
- [ ] "Don't have an account? Sign Up" link
- [ ] "Forgot password?" link (stub for future)
- [ ] Error messages from i18n
- [ ] Loading state during submission
- [ ] Redirect to dashboard on success
- [ ] All labels from i18n

**Code Outline**:
```typescript
export const SigninForm: React.FC = () => {
  const { t } = useTranslation(["auth", "errors"]);
  const [error, setError] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  const { control, handleSubmit, formState: { errors } } = useForm({
    defaultValues: { email: "", password: "" }
  });

  const onSubmit = async (data) => {
    try {
      setIsLoading(true);
      const response = await apiClient.post("/api/auth/signin", data);
      localStorage.setItem("session_token", response.data.session_token);
      window.location.href = "/dashboard";
    } catch (err) {
      setError(t("errors:error_invalid_request"));
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit(onSubmit)}>
      <h1>{t("auth:signin_title")}</h1>

      {error && <div className={styles.error}>{error}</div>}

      <div className={styles.formGroup}>
        <label>{t("auth:email_label")} *</label>
        <Controller
          name="email"
          control={control}
          rules={{ required: t("auth:required_field") }}
          render={({ field }) => <input {...field} type="email" />}
        />
      </div>

      <div className={styles.formGroup}>
        <label>{t("auth:password_label")} *</label>
        <Controller
          name="password"
          control={control}
          rules={{ required: t("auth:required_field") }}
          render={({ field }) => <input {...field} type="password" />}
        />
      </div>

      <button type="submit" disabled={isLoading}>
        {isLoading ? t("auth:signing_in") : t("auth:signin_button")}
      </button>

      <div className={styles.links}>
        <a href="/forgot-password">{t("auth:forgot_password")}</a>
        <span>{t("auth:dont_have_account")}</span>
        <a href="/signup">{t("auth:signup_title")}</a>
      </div>
    </form>
  );
};
```

**Test Case**:
```typescript
test("signs in user and redirects", async () => {
  const mockNavigate = jest.fn();
  render(<SigninForm />);

  fireEvent.change(screen.getByLabelText("Email"), { target: { value: "test@example.com" } });
  fireEvent.change(screen.getByLabelText("Password"), { target: { value: "password123" } });
  fireEvent.click(screen.getByText("Sign In"));

  await waitFor(() => {
    expect(localStorage.getItem("session_token")).toBeTruthy();
  });
});
```

**Bilingual Output**: All labels and messages in selected language

**Story Points**: 6

**Dependencies**: Task 2.2, 2.3, 2.6

---

### Task 3.8: Update Navbar with Auth Links & Language Selector

**Description**: Update existing navbar to show auth state and language selector.

**File**: `website/src/theme/Navbar/index.tsx` (update existing)

**Acceptance Criteria**:
- [ ] Display "Sign Up" and "Sign In" buttons when not authenticated
- [ ] Display user name and "Logout" button when authenticated
- [ ] Add LanguageSelector component (Task 3.1) to navbar
- [ ] Links use i18n for button text
- [ ] Responsive on mobile (dropdown menu or hamburger)
- [ ] "Logout" clears localStorage and redirects to home
- [ ] All text from i18n ("auth:signup_button", etc.)

**Code Outline**:
```typescript
export default function Navbar() {
  const { user, isAuthenticated } = useAuthContext();
  const { t } = useTranslation("auth");

  const handleLogout = () => {
    localStorage.removeItem("session_token");
    window.location.href = "/";
  };

  return (
    <nav className={styles.navbar}>
      {/* Logo */}
      <div className={styles.logo}>
        <Link to="/">Physical AI</Link>
      </div>

      {/* Right side: Language + Auth */}
      <div className={styles.rightSection}>
        <LanguageSelector />

        {isAuthenticated ? (
          <>
            <span className={styles.userName}>{user?.name}</span>
            <button onClick={handleLogout} className={styles.logoutButton}>
              {t("auth:signout_button")}
            </button>
          </>
        ) : (
          <>
            <Link to="/signin" className={styles.signinButton}>
              {t("auth:signin_button")}
            </Link>
            <Link to="/signup" className={styles.signupButton}>
              {t("auth:signup_button")}
            </Link>
          </>
        )}
      </div>
    </nav>
  );
}
```

**Test Case**:
```typescript
test("shows auth links when not authenticated", () => {
  render(<Navbar />);

  expect(screen.getByText("Sign In")).toBeInTheDocument();
  expect(screen.getByText("Sign Up")).toBeInTheDocument();
});

test("shows logout button when authenticated", () => {
  // Mock auth context
  mockAuthContext({ isAuthenticated: true, user: { name: "Test User" } });

  render(<Navbar />);

  expect(screen.getByText("Test User")).toBeInTheDocument();
  expect(screen.getByText("Logout")).toBeInTheDocument();
});

test("language selector works in navbar", () => {
  render(<Navbar />);

  fireEvent.click(screen.getByText("اردو"));

  // Verify language changed
  expect(localStorage.getItem("language_preference")).toBe("ur");
});
```

**Bilingual Output**: Button labels in selected language

**Story Points**: 6

**Dependencies**: Task 2.2, 2.3, Task 3.1, Task 3.7

---

### Task 3.9: Update AuthContext with Language Preference

**Description**: Extend AuthContext to manage language preference syncing.

**File**: `website/src/context/AuthContext.tsx` (update existing)

**Acceptance Criteria**:
- [ ] AuthContext includes `languagePreference` state
- [ ] On login, sync language_preference from profile to localStorage
- [ ] On logout, clear language_preference
- [ ] Expose `setLanguagePreference` function
- [ ] Language preference available to all components via context

**Code Addition**:
```typescript
interface AuthContextType {
  user: User | null;
  profile: UserProfile | null;
  languagePreference: "en" | "ur";
  isAuthenticated: boolean;
  setLanguagePreference: (lang: "en" | "ur") => void;
  logout: () => void;
}

export const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [languagePreference, setLanguagePreference] = useState<"en" | "ur">(
    (localStorage.getItem("language_preference") as "en" | "ur") || "en"
  );

  useEffect(() => {
    // Sync to i18n on change
    i18n.changeLanguage(languagePreference);
  }, [languagePreference]);

  // On login, update language from profile
  useEffect(() => {
    if (profile?.language_preference) {
      setLanguagePreference(profile.language_preference);
    }
  }, [profile]);

  return (
    <AuthContext.Provider value={{ languagePreference, setLanguagePreference, ... }}>
      {children}
    </AuthContext.Provider>
  );
};
```

**Bilingual Output**: N/A (Internal state management)

**Story Points**: 3

**Dependencies**: Task 2.1, existing AuthContext

---

### Task 3.10: Create User Profile Page

**Description**: Display user profile with background info and content access level.

**File**: `website/src/pages/profile.tsx` (new)

**Acceptance Criteria**:
- [ ] Requires authentication (redirect if not logged in)
- [ ] Display user email, name, language preference
- [ ] Display all profile fields (programming_years, languages, ai_ml_level, etc.)
- [ ] Show content access level badge (Beginner → Basic, Intermediate → Intermediate, Advanced → All)
- [ ] Show "Upgrade your profile" section if not advanced
- [ ] Edit button to update profile (TODO for future)
- [ ] All labels from i18n

**Code Outline**:
```typescript
export default function ProfilePage() {
  const { user, profile } = useAuthContext();
  const { t } = useTranslation(["auth", "forms"]);
  const navigate = useNavigate();

  if (!user) {
    navigate("/signin");
    return null;
  }

  const contentAccessMap = {
    beginner: "Beginner Level - Basic & Fundamentals",
    intermediate: "Intermediate Level - Basic + Intermediate + Projects",
    advanced: "Advanced Level - All Content + Capstone + Research",
    none: "No AI/ML Content"
  };

  return (
    <div className={styles.profilePage}>
      <h1>{t("auth:my_profile")}</h1>

      <section className={styles.accountSection}>
        <h2>{t("auth:account_info")}</h2>
        <p>Email: {user.email}</p>
        <p>Name: {user.name}</p>
        <p>Language: {profile?.language_preference === "ur" ? "اردو" : "English"}</p>
      </section>

      <section className={styles.backgroundSection}>
        <h2>{t("forms:software_background_title")}</h2>
        <p>Programming Experience: {t(`forms:programming_years_${profile?.programming_years}`)}</p>
        <p>Languages: {profile?.languages.join(", ")}</p>
        <p>AI/ML Level: {t(`forms:ai_ml_${profile?.ai_ml_level}`)}</p>
      </section>

      <section className={styles.accessSection}>
        <h2>Content Access</h2>
        <div className={styles.accessBadge}>
          {contentAccessMap[profile?.ai_ml_level || "none"]}
        </div>
      </section>

      {profile?.ai_ml_level !== "advanced" && (
        <section className={styles.upgradeSection}>
          <h2>🎯 Upgrade Your Access</h2>
          <p>To unlock advanced content and research papers, update your profile with:</p>
          <ul>
            <li>Advanced AI/ML projects and experience</li>
            <li>Physical humanoid robotics work</li>
            <li>Research publications</li>
          </ul>
          <button onClick={() => alert("Edit profile - coming soon")}>Edit Profile</button>
        </section>
      )}
    </div>
  );
}
```

**Test Case**:
```typescript
test("displays user profile", () => {
  mockAuthContext({
    user: { email: "test@example.com", name: "Test User" },
    profile: { ai_ml_level: "intermediate", languages: ["Python"] }
  });

  render(<ProfilePage />);

  expect(screen.getByText("test@example.com")).toBeInTheDocument();
  expect(screen.getByText("Intermediate Level")).toBeInTheDocument();
});
```

**Bilingual Output**: All labels and descriptions in selected language

**Story Points**: 6

**Dependencies**: Task 2.1, 2.2, 2.3, 2.4, 2.5, Task 3.9

---

### Task 3.11: Create Content Gate Component

**Description**: Component to protect content based on access level.

**File**: `website/src/components/ContentGate.tsx`

**Acceptance Criteria**:
- [ ] Accepts `requiredLevel` prop (beginner | intermediate | advanced)
- [ ] Displays content if user's ai_ml_level is sufficient
- [ ] Shows "Access Denied" message with "Upgrade Profile" link if not sufficient
- [ ] Error message in user's language
- [ ] Shows "Sign in to view" if not authenticated
- [ ] Responsive layout

**Code Outline**:
```typescript
interface ContentGateProps {
  requiredLevel: "beginner" | "intermediate" | "advanced";
  children: React.ReactNode;
}

export const ContentGate: React.FC<ContentGateProps> = ({ requiredLevel, children }) => {
  const { user, profile } = useAuthContext();
  const { t } = useTranslation("auth");

  if (!user) {
    return (
      <div className={styles.gateDenied}>
        <p>🔒 {t("please_sign_in")}</p>
        <a href="/signin">{t("signin_button")}</a>
      </div>
    );
  }

  const levelHierarchy = { beginner: 0, intermediate: 1, advanced: 2 };
  const userLevel = levelHierarchy[profile?.ai_ml_level] ?? -1;

  if (userLevel < levelHierarchy[requiredLevel]) {
    return (
      <div className={styles.gateDenied}>
        <h3>{t("access_denied_title")}</h3>
        <p>{t("access_denied_message", { level: requiredLevel })}</p>
        <a href="/profile">{t("upgrade_profile")}</a>
      </div>
    );
  }

  return <>{children}</>;
};
```

**Test Case**:
```typescript
test("blocks access for beginner user to advanced content", () => {
  mockAuthContext({
    user: { name: "Test" },
    profile: { ai_ml_level: "beginner" }
  });

  render(
    <ContentGate requiredLevel="advanced">
      <div>Secret Content</div>
    </ContentGate>
  );

  expect(screen.getByText("Access Denied")).toBeInTheDocument();
  expect(screen.queryByText("Secret Content")).not.toBeInTheDocument();
});
```

**Bilingual Output**: Access denied messages in selected language

**Story Points**: 5

**Dependencies**: Task 2.2, 2.3, 2.6

---

### Task 3.12: Create Signup & Signin Pages

**Description**: Full pages for signup and signin, integrating components.

**Files**:
- `website/src/pages/signup.tsx`
- `website/src/pages/signin.tsx`

**Acceptance Criteria**:
- [ ] Signup page renders MultiStepSignupForm
- [ ] Signin page renders SigninForm
- [ ] Language selector visible on both pages
- [ ] Both pages styled consistently
- [ ] Responsive layout (mobile-first)
- [ ] Redirect if already logged in
- [ ] SEO meta tags (title, description)
- [ ] All text from i18n

**Code Outline** (signup.tsx):
```typescript
export default function SignupPage() {
  const { isAuthenticated } = useAuthContext();
  const navigate = useNavigate();

  useEffect(() => {
    if (isAuthenticated) {
      navigate("/dashboard");
    }
  }, [isAuthenticated]);

  return (
    <Layout>
      <div className={styles.signupPage}>
        <div className={styles.container}>
          <MultiStepSignupForm />
        </div>
      </div>
    </Layout>
  );
}
```

**Test Case**:
```typescript
test("signup page redirects if already logged in", () => {
  mockAuthContext({ isAuthenticated: true });
  const mockNavigate = jest.fn();

  render(<SignupPage />);

  expect(mockNavigate).toHaveBeenCalledWith("/dashboard");
});
```

**Bilingual Output**: All page content in selected language

**Story Points**: 4

**Dependencies**: Task 3.2, Task 3.7

---

## Phase 4: Integration & Chat Personalization (2 tasks, ~15 SP)

### Task 4.1: Update RAG Chat to Include User Profile Context

**Description**: Modify chat endpoints and client to pass user profile for personalization.

**Files**:
- `website/src/services/apiClient.ts` (update)
- `website/src/hooks/useRAGChat.ts` (update)
- `backend/api/routes/chat.py` (update)

**Acceptance Criteria**:
- [ ] Chat endpoint receives user_id from session
- [ ] Loads user profile from database (cached for 5 min)
- [ ] Passes profile context to RAG chatbot system prompt
- [ ] Chat response acknowledges user background ("Since you're experienced with ROS...")
- [ ] Recommendations filtered by ai_ml_level
- [ ] Frontend hook returns profile data along with messages
- [ ] Chat works for authenticated users, error message for non-authenticated

**Backend Changes**:
```python
@router.post("/api/chat-stream")
async def chat_stream(
    request: ChatRequest,
    user_id: str = Depends(get_current_user_id),
    user_service: UserService = Depends(),
):
  # Get user profile
  profile = await user_service.get_user_profile(user_id)

  # Build context
  profile_context = build_profile_context(profile)

  # Inject into system prompt
  system_prompt = f"""
  User Background: {profile_context}

  Tailor recommendations to their level: {profile.ai_ml_level}
  Acknowledge their experience where relevant.
  """

  # Stream response with personalization
  async for chunk in chatbot.stream(system_prompt=system_prompt, ...):
    yield chunk
```

**Frontend Changes**:
```typescript
async function sendQuery(query: string) {
  const profile = await apiClient.getUserProfile();

  const response = await apiClient.chatStream({
    query,
    user_profile: profile
  });

  // Process stream with profile context
}
```

**Test Case**:
```python
# Verify profile passed to system prompt
response = client.post(
    "/api/chat-stream",
    json={"query": "What is ROS?"},
    headers={"Authorization": f"Bearer {token}"}
)

# Check response includes profile-aware recommendation
assert "Since you have" in response.text or "Python" in response.text
```

**Bilingual Output**: Chat responses respect user's language preference (passed in profile)

**Story Points**: 8

**Dependencies**: Task 1.4, 1.5, 1.9, Task 3.2

---

### Task 4.2: Update Chat Component to Show Personalization Info

**Description**: Display personalization indicators in chat (e.g., "Filtered for Advanced Users").

**File**: `website/src/components/RAGChat/ChatWindow.tsx` (update)

**Acceptance Criteria**:
- [ ] Show profile badge under chat title (e.g., "Advanced Level - All Content Unlocked")
- [ ] Display note if response is filtered (e.g., "⚙️ Recommendations filtered for Advanced users")
- [ ] Show warning if user is near level requirement (e.g., "⚠️ Some advanced content not shown")
- [ ] Language preference icon next to profile name
- [ ] All labels from i18n

**Code Outline**:
```typescript
export const ChatWindow: React.FC<ChatWindowProps> = ({ messages, profile }) => {
  const { t } = useTranslation("chat");

  const profileBadges = {
    beginner: t("beginner_profile"),
    intermediate: t("intermediate_profile"),
    advanced: t("advanced_profile")
  };

  return (
    <div className={styles.chatWindow}>
      <div className={styles.profileInfo}>
        <span className={styles.badge}>{profileBadges[profile?.ai_ml_level]}</span>
        <span className={styles.language}>
          {profile?.language_preference === "ur" ? "اردو" : "EN"}
        </span>
      </div>

      {messages.map((msg) => (
        <div key={msg.id} className={styles.message}>
          {msg.content}
          {msg.filtered && (
            <div className={styles.filterInfo}>
              ⚙️ {t("chat:filtered_for_level", { level: profile?.ai_ml_level })}
            </div>
          )}
        </div>
      ))}
    </div>
  );
};
```

**Test Case**:
```typescript
test("displays profile personalization info", () => {
  const profile = { ai_ml_level: "advanced", language_preference: "en" };

  render(<ChatWindow messages={[]} profile={profile} />);

  expect(screen.getByText("Advanced Level - All Content Unlocked")).toBeInTheDocument();
});
```

**Bilingual Output**: All personalization messages in user's language

**Story Points**: 5

**Dependencies**: Task 3.2, Task 4.1

---

## Phase 5: Testing & Deployment (5 tasks, ~30 SP)

### Task 5.1: Create Unit Tests for Auth Services

**Description**: Comprehensive unit tests for backend auth services.

**Files**:
- `backend/tests/test_auth_service.py` (password hashing, sessions)
- `backend/tests/test_user_service.py` (CRUD operations)
- `backend/tests/test_audit_service.py` (logging)

**Acceptance Criteria**:
- [ ] Test password hashing (bcrypt, different salts)
- [ ] Test password verification (correct/incorrect)
- [ ] Test session creation and verification
- [ ] Test user CRUD operations (create, read, update)
- [ ] Test profile CRUD operations
- [ ] Test audit logging
- [ ] All tests passing
- [ ] Coverage > 80% for auth services

**Test Cases**:
- `test_bcrypt_password_hashing`: Verify bcrypt with 12 rounds
- `test_password_verification`: Correct and incorrect passwords
- `test_session_token_uniqueness`: Each session has unique token
- `test_user_profile_creation`: User + profile atomic transaction
- `test_audit_log_capture`: Signup/signin logged with timestamps
- `test_profile_update`: Only specified fields updated

**Story Points**: 8

**Dependencies**: Task 1.4, 1.5, 1.6

---

### Task 5.2: Create Integration Tests for Auth Endpoints

**Description**: Full flow testing of signup/signin/session endpoints.

**File**: `backend/tests/test_auth_flow.py`

**Acceptance Criteria**:
- [ ] Test complete signup flow (all 4 steps)
- [ ] Test signin with valid/invalid credentials
- [ ] Test session verification
- [ ] Test signout and token revocation
- [ ] Test profile retrieval
- [ ] Test content gating (403 for unauthorized access)
- [ ] Test error cases (duplicate email, invalid input)
- [ ] All tests passing
- [ ] Coverage > 80% for endpoints

**Test Cases**:
- `test_signup_complete_flow`: Email → Password → Profile → Redirect
- `test_signin_with_valid_credentials`: Returns session token
- `test_signin_with_invalid_password`: Returns 401
- `test_session_verification`: Valid token returns user
- `test_content_gating`: Beginner can't access advanced
- `test_profile_returned_on_session`: Profile included in session response

**Story Points**: 8

**Dependencies**: Task 1.7, 1.8, 1.9, 1.11

---

### Task 5.3: Create E2E Tests for Signup/Signin Flow

**Description**: Full browser-based tests using Playwright.

**File**: `website/tests/auth-flow.e2e.test.ts`

**Acceptance Criteria**:
- [ ] Test English signup (4 steps, all fields)
- [ ] Test Urdu signup (verify UI in Urdu)
- [ ] Test signup validation errors (in both languages)
- [ ] Test signin flow
- [ ] Test language toggle persistence
- [ ] Test logout
- [ ] Test content gating (beginner can't access capstone)
- [ ] Test profile page display
- [ ] All tests passing

**Test Cases**:
- `test_complete_signup_english`: Fill all 4 steps, verify redirect
- `test_signup_with_urdu_language`: Toggle to Urdu, verify labels in Urdu
- `test_signup_validation_email_exists`: Duplicate email shows error
- `test_signin_and_logout`: Sign in, verify session, logout
- `test_language_persistence`: Signup in English, return later, still English
- `test_content_gating_beginner_blocks_capstone`: Beginner sees "Access Denied"

**Story Points**: 10

**Dependencies**: Task 3.2, 3.3, 3.4, 3.5, 3.6, 3.7

---

### Task 5.4: Security & Vulnerability Testing

**Description**: Assess security against OWASP Top 10 and create fixes.

**File**: `backend/tests/test_security.py`

**Acceptance Criteria**:
- [ ] SQL injection test (Drizzle ORM prevents)
- [ ] XSS test (React escaping verified)
- [ ] CSRF test (SameSite cookies verified)
- [ ] Password strength enforcement (12+ chars, bcrypt)
- [ ] Session security (httpOnly, Secure, SameSite)
- [ ] No hardcoded secrets
- [ ] Rate limiting on signup/signin
- [ ] Email verification prevents spam
- [ ] All tests passing

**Test Cases**:
- `test_sql_injection_prevention`: Malicious SQL in email field is escaped
- `test_xss_prevention`: HTML tags in name field are escaped
- `test_csrf_protection`: Request without proper tokens rejected
- `test_weak_password_rejected`: < 12 chars rejected
- `test_bcrypt_hashing`: Passwords never stored in plaintext
- `test_rate_limiting`: > 5 signin attempts blocked for 15 min
- `test_email_verification_prevents_dummy_emails`: Invalid domains blocked

**Story Points**: 8

**Dependencies**: All auth tasks

---

### Task 5.5: Create Deployment Guide & Documentation

**Description**: Write deployment procedures, monitoring, and rollback guides.

**Files**:
- `docs/deployment.md` (deployment procedures)
- `docs/monitoring.md` (alerting, dashboards)
- `docs/runbooks.md` (common operational tasks)
- `.env.example` (env var template)

**Acceptance Criteria**:
- [ ] Pre-deployment checklist (tests, migration, backup)
- [ ] Step-by-step deployment instructions
- [ ] Database migration verification
- [ ] Rollback procedures
- [ ] Monitoring and alerting setup
- [ ] Log aggregation configuration
- [ ] Runbook for common issues (failed signup, session issues, etc.)
- [ ] Emergency contacts and escalation path
- [ ] All documentation complete and reviewed

**Sections**:
1. Pre-Deployment Checklist
   - Run all tests
   - Database backup
   - Feature flags configured
   - Env vars validated

2. Deployment Steps
   - Frontend build
   - Backend deployment
   - Database migration
   - Health checks

3. Post-Deployment
   - Monitor error rates
   - Test signup flow manually
   - Verify email verification (if enabled)
   - Check audit logs

4. Monitoring
   - Auth success rate (target > 99%)
   - Signup completion rate (target > 80%)
   - Form submission latency (target < 1s)
   - Error rate by endpoint

5. Rollback
   - Database rollback procedure
   - Feature flag disabling
   - Reverting to previous backend version

**Story Points**: 6

**Dependencies**: All backend and frontend tasks

---

## Summary Table

| Phase | Task | Title | SP | Status |
|-------|------|-------|----|----|
| 1 | 1.1 | Drizzle ORM Schema | 5 | Ready |
| 1 | 1.2 | Database Migration | 3 | Ready |
| 1 | 1.3 | Better Auth Config | 5 | Ready |
| 1 | 1.4 | User Service | 8 | Ready |
| 1 | 1.5 | Auth Service | 6 | Ready |
| 1 | 1.6 | Audit Service | 4 | Ready |
| 1 | 1.7 | Signup Endpoint | 8 | Ready |
| 1 | 1.8 | Signin Endpoint | 6 | Ready |
| 1 | 1.9 | Session Endpoint | 4 | Ready |
| 1 | 1.10 | Signout Endpoint | 3 | Ready |
| 1 | 1.11 | Content Gating Middleware | 6 | Ready |
| 1 | 1.12 | Profile Update Endpoint | 5 | Ready |
| 1 | 1.13 | Email Verification | 5 | Ready |
| **Phase 1 Total** | | | **69 SP** | |
| | | | | |
| 2 | 2.1 | i18n Config | 3 | Ready |
| 2 | 2.2 | English Auth Translations | 4 | Ready |
| 2 | 2.3 | Urdu Auth Translations | 4 | Ready |
| 2 | 2.4 | English Form Translations | 5 | Ready |
| 2 | 2.5 | Urdu Form Translations | 5 | Ready |
| 2 | 2.6 | Error Translations | 4 | Ready |
| 2 | 2.7 | Chat Translations | 3 | Ready |
| 2 | 2.8 | Validate Translations | 3 | Ready |
| **Phase 2 Total** | | | **31 SP** | |
| | | | | |
| 3 | 3.1 | Language Selector | 5 | Ready |
| 3 | 3.2 | MultiStep Signup Form | 10 | Ready |
| 3 | 3.3 | Signup Step 1 | 8 | Ready |
| 3 | 3.4 | Signup Step 2 | 8 | Ready |
| 3 | 3.5 | Signup Step 3 | 8 | Ready |
| 3 | 3.6 | Signup Step 4 | 6 | Ready |
| 3 | 3.7 | Signin Form | 6 | Ready |
| 3 | 3.8 | Update Navbar | 6 | Ready |
| 3 | 3.9 | Update AuthContext | 3 | Ready |
| 3 | 3.10 | Profile Page | 6 | Ready |
| 3 | 3.11 | Content Gate Component | 5 | Ready |
| 3 | 3.12 | Signup/Signin Pages | 4 | Ready |
| **Phase 3 Total** | | | **75 SP** | |
| | | | | |
| 4 | 4.1 | Chat Profile Context | 8 | Ready |
| 4 | 4.2 | Chat Personalization UI | 5 | Ready |
| **Phase 4 Total** | | | **13 SP** | |
| | | | | |
| 5 | 5.1 | Unit Tests | 8 | Ready |
| 5 | 5.2 | Integration Tests | 8 | Ready |
| 5 | 5.3 | E2E Tests | 10 | Ready |
| 5 | 5.4 | Security Testing | 8 | Ready |
| 5 | 5.5 | Deployment Guide | 6 | Ready |
| **Phase 5 Total** | | | **40 SP** | |
| | | | | |
| **GRAND TOTAL** | | | **228 SP** | |

---

## Critical Dependencies & Order

**Must Complete Before Phase 2 Starts**:
- Task 1.1 (Schema definition)
- Task 1.2 (Migration)
- Task 1.3 (Better Auth config)

**Must Complete Before Phase 3 Starts**:
- All Phase 1 tasks (13 tasks)
- Tasks 2.1, 2.2, 2.3 (i18n setup and auth translations)

**Can Parallelize**:
- Phase 1 tasks 1.4-1.13 (after 1.1-1.3)
- Phase 2 tasks (translations can be done in parallel)
- Phase 3 tasks (after Phase 2 complete)
- Phase 4 depends on Phase 1 & 3
- Phase 5 depends on all others

---

## Bilingual Output Summary

**All Form Labels in Both Languages**:
- ✓ Email, Password, Confirm Password, Name (English + اردو)
- ✓ Programming years dropdown (English + اردو)
- ✓ Language options (with code names: Python, JavaScript, etc.)
- ✓ AI/ML levels (None/Beginner/Intermediate/Advanced + کوئی نہیں/ابتدائی/درمیانی/ماہر)
- ✓ ROS experience (English + اردو)
- ✓ Hardware platforms (English + اردو)
- ✓ Humanoid experience (English + اردو)
- ✓ Simulation tools (English + اردو)
- ✓ All buttons (Next/Back/Complete/Sign In/Out + اگلا/واپس/تکمیل/سائن اِن/آؤٹ)

**All Error Messages in Both Languages**:
- ✓ Invalid email (English + اردو)
- ✓ Email already exists (English + اردو)
- ✓ Password too short (English + اردو)
- ✓ Passwords don't match (English + اردو)
- ✓ Field required (English + اردو)
- ✓ Server errors (English + اردو)

**All Chat Messages in Both Languages**:
- ✓ Personalization indicators (English + اردو)
- ✓ Sign in prompts (English + اردو)
- ✓ Access denied messages (English + اردو)
- ✓ Content gating messages (English + اردو)

---

**Plan Status**: Ready for Implementation
**Total Story Points**: 228 SP (approx 5-6 weeks for small team of 2-3 developers)
**Test Coverage Target**: > 80% overall, > 90% for auth services
**Bilingual Completeness**: 100% of user-facing text in English + Urdu

---
