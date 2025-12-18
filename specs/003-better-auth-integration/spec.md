# Feature Specification: Better Auth Integration & User Authentication

**Feature Branch**: `003-better-auth-integration`
**Created**: 2025-12-18
**Status**: Draft
**Input**: User description: "Extend user schema with additionalFields (e.g., softwareBackground: string/JSON, hardwareBackground: string/JSON, experienceLevel: string). Custom Sign Up flow: Multi-step form in Docusaurus (React component) asking background questions after basic email/password. Secure: Better Auth handles hashing, sessions, email verification optional. Integration: Add Auth pages (SignIn/SignUp) in Docusaurus, global header with login/logout, floating chatbot aware of user profile."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - User Registration with Background Questions (Priority: P1)

New users visiting the Docusaurus site can create an account through a multi-step sign-up process that collects email/password and then asks about their technical background (software experience, hardware experience, robotics interest, experience level). After completing onboarding, they are logged in and can access authenticated features.

**Why this priority**: Account creation is the foundation for all authenticated features. Without this, users cannot log in and the entire auth system is blocked. This is the critical path feature.

**Independent Test**: Can be fully tested by:
1. Navigating to the sign-up page
2. Entering email and password
3. Confirming email or skipping verification
4. Answering all background questions
5. Submitting the form
6. Verifying the account is created in the database
7. Checking that user is logged in and redirected to home/dashboard
8. Verifying profile fields are saved correctly

**Acceptance Scenarios**:

1. **Given** user is on the sign-up page, **When** user enters valid email and password, **Then** form advances to step 2 (background questions)
2. **Given** user is on background questions step, **When** user selects programming backgrounds, frameworks, hardware experience, robotics interest, and experience level, **Then** all fields are collected
3. **Given** user completes all steps, **When** user submits the form, **Then** new user account is created in the database with all fields stored securely
4. **Given** account is created, **When** form submits, **Then** user is automatically logged in and redirected to home page or dashboard
5. **Given** user tries to sign up with existing email, **When** submitting the form, **Then** system returns error "Account with this email already exists"
6. **Given** user enters weak password (less than 8 characters), **When** form validates, **Then** system shows error and prevents submission
7. **Given** user skips optional email verification step (if optional), **When** user submits, **Then** account is created without email verification

---

### User Story 2 - User Login with Session Management (Priority: P1)

Existing users can log in using their email and password. The system creates a secure session and authenticates them, allowing access to protected features like the RAG chatbot with user-aware context.

**Why this priority**: Login is essential for accessing the system after initial registration. Without session management, users are logged out on refresh and cannot maintain authenticated state. This is core to the application experience.

**Independent Test**: Can be fully tested by:
1. Visiting the sign-in page
2. Entering valid email and password
3. Submitting the login form
4. Verifying user is logged in (auth token/session is created)
5. Checking that user can access protected pages
6. Refreshing the page and verifying session persists
7. Testing with invalid credentials and verifying rejection

**Acceptance Scenarios**:

1. **Given** user is on the sign-in page, **When** user enters correct email and password, **Then** login succeeds and user is redirected to home page
2. **Given** login succeeds, **When** session is created, **Then** auth token is stored securely (httpOnly cookie or secure localStorage with CSRF protection)
3. **Given** user is logged in, **When** user refreshes the page, **Then** session persists and user remains logged in
4. **Given** user is logged in, **When** user navigates to protected pages, **Then** user can access those pages without re-authenticating
5. **Given** user enters incorrect password, **When** user submits login form, **Then** system returns error "Invalid email or password"
6. **Given** user enters non-existent email, **When** user submits login form, **Then** system returns error "Invalid email or password" (same as wrong password to prevent email enumeration)
7. **Given** user wants to stay logged in, **When** user closes and reopens browser, **Then** user is still logged in (session/token persists)

---

### User Story 3 - User Logout and Session Invalidation (Priority: P1)

Logged-in users can click a logout button in the header/navbar to end their session. After logout, they are redirected to the home page and cannot access protected features without logging in again.

**Why this priority**: Session management requires the ability to explicitly end sessions. Without logout, users cannot switch accounts or end their session securely. This is essential for multi-device/shared computer scenarios.

**Independent Test**: Can be fully tested by:
1. Being logged in
2. Clicking the logout button in the header
3. Verifying redirect to home page
4. Attempting to access a protected page
5. Verifying user is redirected to sign-in page
6. Checking that auth token is cleared

**Acceptance Scenarios**:

1. **Given** user is logged in, **When** user clicks the "Logout" button in the header, **Then** user's session is invalidated
2. **Given** session is invalidated, **When** logout completes, **Then** user is redirected to the home page
3. **Given** user has logged out, **When** user tries to access a protected page, **Then** user is redirected to sign-in page
4. **Given** user has logged out, **When** user closes and reopens the browser, **Then** user is not logged in

---

### User Story 4 - View and Update User Profile (Priority: P2)

Logged-in users can access their profile page to view and edit their account information, including email, name, and background preferences. Changes are saved to the database and reflected immediately.

**Why this priority**: Profile management allows users to update their information after initial sign-up. While important, it's not required for MVP. Users can function without editing their profile immediately after registration.

**Independent Test**: Can be fully tested by:
1. Navigating to the profile page (when logged in)
2. Viewing current profile information
3. Editing one or more fields
4. Saving changes
5. Verifying changes appear in the database
6. Refreshing the page and confirming changes persisted

**Acceptance Scenarios**:

1. **Given** user is logged in, **When** user navigates to /profile, **Then** profile page loads and displays current user information
2. **Given** user is viewing their profile, **When** user edits a field (e.g., programming backgrounds), **Then** changes are reflected in the form
3. **Given** user makes changes, **When** user clicks "Save Profile", **Then** changes are saved to the database
4. **Given** changes are saved, **When** save completes, **Then** user sees a success message (toast/notification)
5. **Given** user saves changes, **When** user refreshes the page, **Then** updated profile information is still displayed
6. **Given** user tries to save with invalid data, **When** validation fails, **Then** system shows error messages and prevents save

---

### User Story 5 - Chatbot Aware of User Profile (Priority: P2)

The RAG chatbot has access to the logged-in user's profile information (background, experience level, etc.) and can tailor responses to match the user's expertise level and interests. Anonymous users get generic responses.

**Why this priority**: Personalization enhances UX but is not required for MVP. The chatbot works without this; profile awareness is an enhancement. Implement after core auth and chatbot are working.

**Independent Test**: Can be fully tested by:
1. Logging in as a beginner user
2. Asking a question to the chatbot
3. Verifying response uses beginner-appropriate language
4. Logging in as an advanced user
5. Asking the same question
6. Verifying response uses more technical language
7. Logging out and asking the same question
8. Verifying response is generic (no personalization)

**Acceptance Scenarios**:

1. **Given** user is logged in with experience_level: "beginner", **When** user asks a question, **Then** chatbot response uses simplified language and explains concepts
2. **Given** user is logged in with experience_level: "advanced", **When** user asks the same question, **Then** chatbot response assumes prior knowledge and uses technical terminology
3. **Given** user is logged in with specific hardwareExperience, **When** relevant questions are asked, **Then** chatbot can reference or personalize around that background
4. **Given** user is not logged in, **When** user asks a question, **Then** chatbot provides generic response without personalization
5. **Given** chatbot has access to user profile, **When** generating response, **Then** profile data is passed securely with request (no sensitive data exposed)

---

### User Story 6 - Persistent Login State in Header/Navbar (Priority: P2)

The Docusaurus header/navbar displays the current login state: shows "Sign In" / "Sign Up" buttons for anonymous users, and shows user's name with a logout button and profile link for logged-in users.

**Why this priority**: UI for login state improves usability but is not critical for MVP. Core auth features work without polished UI. This can be implemented after core flows work.

**Independent Test**: Can be fully tested by:
1. Visiting site as anonymous user
2. Verifying "Sign In" and "Sign Up" buttons appear
3. Logging in
4. Verifying header shows logged-in state with user name
5. Verifying logout button appears
6. Clicking logout
7. Verifying header returns to anonymous state

**Acceptance Scenarios**:

1. **Given** user is not logged in, **When** user views the header, **Then** "Sign In" and "Sign Up" buttons are displayed
2. **Given** user is logged in, **When** user views the header, **Then** user's name is displayed with a dropdown menu
3. **Given** user is logged in, **When** user views the header dropdown, **Then** "Profile" and "Logout" options are visible
4. **Given** user is logged in, **When** user clicks their name, **Then** dropdown opens/closes smoothly
5. **Given** user clicks "Profile" link, **When** navigation occurs, **Then** user is taken to /profile page

---

### Edge Cases

- What happens when a user's session token expires mid-request? (Graceful refresh or redirect to login)
- How does the system handle concurrent login attempts from multiple devices? (All should succeed; Better Auth handles session isolation)
- What if email verification is optional but enabled later? (Existing users should not be forced to verify)
- What if a user tries to register with an email address that contains special characters? (Validate and either normalize or reject with clear error)
- What if the database connection fails during user creation? (Return 500 error and ensure transaction rolls back)
- How are user sessions cleared when the database is wiped during development? (Use feature flags or manual cleanup scripts)

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow users to register with email and password through a multi-step sign-up form
- **FR-002**: System MUST validate email format, password strength (minimum 8 characters), and required fields at both frontend and backend
- **FR-003**: System MUST collect user background information (programming languages, frameworks, hardware experience, robotics interest, experience level) during sign-up
- **FR-004**: System MUST hash passwords securely using bcrypt or Argon2 before storing (NEVER store plaintext passwords)
- **FR-005**: System MUST generate and return secure session tokens or auth cookies after successful login
- **FR-006**: System MUST validate session tokens on every authenticated request and reject expired/invalid tokens
- **FR-007**: System MUST support logout by invalidating the user's session/token
- **FR-008**: System MUST persist user sessions across browser refresh and close/reopen (using secure cookies or localStorage with token refresh)
- **FR-009**: System MUST prevent duplicate account registration (return error if email already exists)
- **FR-010**: System MUST allow authenticated users to view their profile information
- **FR-011**: System MUST allow authenticated users to update their profile (name, email, background preferences)
- **FR-012**: System MUST return 401 Unauthorized for requests to protected endpoints without valid authentication
- **FR-013**: System MUST protect sensitive user data (passwords, session tokens) with appropriate security measures
- **FR-014**: The chatbot MUST have read access to logged-in user's profile to tailor responses (no modification of profile from chatbot)
- **FR-015**: System MUST log authentication events (signup, login, logout) for security auditing (optional but recommended)

### Key Entities

- **User**: Represents a user account with id, email, password_hash, name, created_at, updated_at
- **UserProfile**: Extends User with background information (programming_backgrounds, frameworks_known, hardware_experience, robotics_interest, experience_level, completed_onboarding)
- **Session**: Represents an active user session with user_id, token, expires_at, created_at (Better Auth may handle this internally)
- **EmailVerificationToken** (optional): If email verification is enabled, stores verification tokens with email and expiration

### Non-Functional Requirements

- **Performance**: Sign-up and login should complete in under 3 seconds (99th percentile)
- **Availability**: Auth service should have 99.9% uptime SLA
- **Security**: All passwords must be hashed; session tokens must be cryptographically secure; HTTPS required in production
- **Scalability**: System should support 10,000+ concurrent authenticated users
- **Auditability**: All auth events should be loggable for compliance and security review
- **Recovery**: Session invalidation and password resets should be reversible (for admin recovery scenarios)

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete sign-up in under 2 minutes (multi-step form is not onerous)
- **SC-002**: Users can log in successfully in under 1 minute
- **SC-003**: 100% of user passwords are hashed before storage (verified through code review)
- **SC-004**: Session tokens persist across page refresh with 100% reliability
- **SC-005**: Logged-in users cannot access protected endpoints after logout
- **SC-006**: System handles 100 concurrent sign-ups without timeouts or data loss
- **SC-007**: Profile updates are reflected in the chatbot's user-aware responses within 2 seconds
- **SC-008**: Auth integration doesn't increase page load time by more than 500ms
- **SC-009**: All authentication endpoints return consistent, clear error messages (no server stack traces exposed)

---

## Known Constraints

- **Better Auth Version**: Using `better-auth@^1.4.5` (already installed in frontend package.json)
- **Database**: PostgreSQL (Neon) is the source of truth; Qdrant is only for vector embeddings
- **Password Policy**: Minimum 8 characters (can be enhanced later with uppercase, numbers, special characters if needed)
- **Email Verification**: Optional (can be disabled to speed up signup during MVP)
- **Rate Limiting**: Not required for MVP but should be considered for production
- **CORS**: Already configured in FastAPI; auth endpoints must be included in CORS allowlist

---

## Architecture Decisions

### Backend Tech Stack

- **Framework**: FastAPI (already in use)
- **Database**: PostgreSQL (Neon) for users, sessions, profiles
- **Password Hashing**: bcrypt via `passlib[bcrypt]`
- **JWT/Sessions**: Use `python-jose` for JWT OR use Better Auth's built-in session handling if compatible with FastAPI backend

**Decision Rationale**: Better Auth has JavaScript/TypeScript client library optimized for frontend. For FastAPI backend, we'll either:
1. Use Better Auth's HTTP-based endpoints if available (requires Better Auth server or adapter)
2. Implement auth backend in FastAPI following Better Auth's schema/API contracts
3. Use a separate Better Auth-compatible library like `authjs` compatible backend

This will be clarified during implementation planning phase.

### Frontend Architecture

- **Auth State**: React Context (AuthContext already exists) with Better Auth client integration
- **Protected Routes**: ProtectedRoute component (already exists) checks auth status before rendering
- **Form State**: react-hook-form (already in use for signup/signin forms)
- **Token Storage**: Secure HTTP-only cookies (if using FastAPI sessions) OR localStorage with token refresh (if using JWT)

**Decision Rationale**: Frontend already has scaffolding in place (SignIn/SignUp pages, AuthContext, ProtectedRoute). We'll integrate Better Auth's client library into the existing architecture.

### Multi-Step Form Flow

```
Step 1: Email & Password
  ↓ (validate email format, password strength)
Step 2: Background Questions
  ↓ (collect programming backgrounds, frameworks, hardware, robotics interest, experience level)
Step 3: (Optional) Email Verification
  ↓ (send verification link or skip)
Account Created → Auto-login → Redirect to home/dashboard
```

**Decision Rationale**: Multi-step avoids form overload and allows step-by-step validation/error feedback. Email verification can be optional to reduce friction.

---

## Integration Points

### Frontend to Backend

1. **POST /api/auth/signup**: Register new user
   - Request: `{ email, password, name, programmingBackgrounds, frameworksKnown, hardwareExperience, roboticsInterest, experienceLevel }`
   - Response: `{ user: { id, email, name, ... }, token: "..." }` or `{ success: true }`

2. **POST /api/auth/signin**: Authenticate user
   - Request: `{ email, password }`
   - Response: `{ user: { id, email, name, ... }, token: "..." }` or `{ success: true }`

3. **POST /api/auth/signout**: Logout user
   - Request: (may be empty or include token for session invalidation)
   - Response: `{ success: true }`

4. **GET /api/auth/session**: Get current authenticated user
   - Request: (token in header or cookie)
   - Response: `{ user: { id, email, name, profileFields... } }` or `{ error: "Unauthorized" }`

5. **PUT /api/users/profile**: Update user profile
   - Request: `{ name?, programmingBackgrounds?, ...}`
   - Response: `{ user: { updated fields } }`

### Chatbot Integration

The RAG chatbot (POST /api/chat, POST /api/chat-stream) should receive the authenticated user's profile in the request and use it to tailor responses.

**Modified Request Structure**:
```json
{
  "query": "...",
  "user": {
    "id": "...",
    "experience_level": "...",
    "hardware_experience": [...],
    ...
  }
}
```

### Header/Navbar Integration

The Docusaurus navbar should display login/logout UI based on AuthContext state (already in progress via ProtectedRoute and AuthContext hooks).

---

## Deployment & DevOps

### Environment Variables

Backend (.env):
```
DATABASE_URL=postgresql://...  # Already set in Neon
AUTH_SECRET=<random-jwt-secret>  # NEW
PASSWORD_HASH_ROUNDS=12  # NEW
```

Frontend (.env):
```
REACT_APP_API_URL=http://localhost:8001  # Already set
```

### Database Migrations

Schema migrations will be created to add:
- `users` table
- `user_profiles` table (or extend `users` with columns)
- `sessions` table (if using server-side sessions instead of JWT)

Migrations should be version-controlled and applied as part of deployment.

### Testing Strategy

- **Unit Tests**: Password hashing, token generation, validation functions
- **Integration Tests**: Full signup/login/logout flows
- **E2E Tests**: User registration through chatbot access with profile awareness

---

## Out of Scope (MVP)

- Two-factor authentication (2FA)
- Social login (Google, GitHub, etc.)
- Password reset / forgot password flow (can be added later)
- Role-based access control (RBAC) beyond basic authenticated/anonymous
- API key management for external integrations
- User deletion / account deactivation
- Email verification (marked as optional; can be added post-MVP)
- Rate limiting on auth endpoints (important for production; consider adding after MVP)
- OAuth2 provider functionality (this app only consumes auth, doesn't provide it)

---

## Next Steps

1. **Clarify Better Auth Backend Setup**: Confirm whether to use Better Auth's Python server adapter, a compatible library, or implement custom backend
2. **Review and Approve Specification**: Get stakeholder sign-off on this spec before proceeding to implementation plan
3. **Create Implementation Plan**: Detail the technical approach, file structure, and dependencies
4. **Create Task Breakdown**: Generate specific, testable tasks for engineers
5. **Begin Implementation**: Start with database schema, then backend endpoints, then frontend integration
