---
description: "Task list for Better Auth Integration & User Authentication feature"
---

# Tasks: Better Auth Integration & User Authentication

**Input**: Design documents from `/specs/003-better-auth-integration/`
**Prerequisites**: plan.md (✅ complete), spec.md (✅ complete)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing. Tests are included to ensure quality.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and directory structure

- [ ] T001 [P] Create backend auth module directories in `backend/api/routes/`, `backend/api/services/`, `backend/api/schemas/`, `backend/api/middleware/`
- [ ] T002 [P] Create frontend auth component directories in `website/src/components/auth/`, `website/src/hooks/`
- [ ] T003 [P] Create database migration scripts directory in `backend/database/migrations/`
- [ ] T004 [P] Update `backend/requirements.txt` with new dependencies: `passlib[bcrypt]`, `python-jose[cryptography]`, `SQLAlchemy` (if not already present)
- [ ] T005 Verify `website/package.json` has `better-auth@^1.4.5`, `react-hook-form@^7.68`, `axios@^1.13` already installed

**Checkpoint**: Project structure ready - foundational phase can begin

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database Schema & Migrations

- [ ] T006 Create database migration SQL script `backend/database/migrations/001_create_users_table.sql` with:
  - `users` table (id, email, password_hash, name, created_at, updated_at)
  - `user_profiles` table (user_id FK, programming_backgrounds JSON, frameworks_known JSON, hardware_experience JSON, robotics_interest STRING, experience_level ENUM, completed_onboarding BOOLEAN)
  - `sessions` table (id, user_id FK, token, expires_at, created_at) [if using server-side sessions]
  - Include indexes on email (unique), user_id
  - Include foreign key constraints with CASCADE delete

- [ ] T007 Implement database migration runner in `backend/database/postgres_client.py`:
  - Function to apply migration scripts on startup
  - Function to verify schema after migration
  - Rollback capability (optional but recommended)

### Pydantic Models & Schemas

- [ ] T008 [P] Create `backend/api/schemas/auth.py` with:
  - `SignUpRequest` model (email, password, name, programming_backgrounds, frameworks_known, hardware_experience, robotics_interest, experience_level)
  - `SignInRequest` model (email, password)
  - `AuthResponse` model (user, token, expires_at)
  - `UserResponse` model (id, email, name, profile fields, created_at)
  - Validation for password strength (min 8 chars) and email format

- [ ] T009 [P] Create `backend/api/schemas/user.py` with:
  - `ProfileUpdateRequest` model (name, programming_backgrounds, frameworks_known, hardware_experience, robotics_interest, experience_level)
  - `ProfileResponse` model with all user profile fields

- [ ] T010 [P] Create `backend/api/models.py` extensions (if not already Pydantic):
  - SQLAlchemy ORM models for User, UserProfile, Session (if using)
  - Column definitions matching database schema

### Auth Service & Utilities

- [ ] T011 Create `backend/api/services/auth_service.py` with:
  - `hash_password(password: str) -> str` function using bcrypt
  - `verify_password(password: str, hash: str) -> bool` function
  - `generate_token(user_id: str, expires_hours: int = 24) -> tuple[str, datetime]` function using python-jose
  - `verify_token(token: str) -> dict[str, any]` function to decode JWT and validate
  - Error classes: `InvalidCredentials`, `UserNotFound`, `UserAlreadyExists`

- [ ] T012 Create `backend/api/services/user_service.py` with:
  - `create_user(email, password, name, profile_data)` -> User (writes to users and user_profiles tables)
  - `get_user_by_email(email)` -> User or None
  - `get_user_by_id(user_id)` -> User with profile
  - `update_user_profile(user_id, profile_data)` -> User
  - All methods handle database transactions and errors

- [ ] T013 Create `backend/api/services/session_service.py` with:
  - `create_session(user_id, token, expires_at)` -> Session (if using server-side sessions)
  - `validate_session(token)` -> user_id or None
  - `invalidate_session(token)` -> bool (for logout)
  - Optional: session cleanup task for expired sessions

### Auth Middleware

- [ ] T014 Create `backend/api/middleware/auth.py` with:
  - `AuthMiddleware` class that:
    - Extracts JWT token from Authorization header (Bearer token)
    - Validates token using `verify_token()` from auth_service
    - Attaches `current_user` to request scope on success
    - Returns 401 Unauthorized on invalid/expired token
  - `get_current_user()` dependency function for route handlers

### Config & Dependencies

- [ ] T015 Update `backend/api/config.py`:
  - Add `AUTH_SECRET: str` setting (read from env, generate random if not set)
  - Add `JWT_ALGORITHM: str = "HS256"`
  - Add `JWT_EXPIRATION_HOURS: int = 24`
  - Add `PASSWORD_HASH_ROUNDS: int = 12`
  - Add `ENABLE_EMAIL_VERIFICATION: bool = False` (disabled for MVP)

- [ ] T016 Update `backend/api/app.py`:
  - Import and add `AuthMiddleware` to FastAPI app
  - Ensure CORS settings allow auth endpoints and auth headers
  - Verify error handlers return proper JSON responses with status codes

### Frontend Auth Infrastructure

- [ ] T017 Update `website/src/context/AuthContext.tsx`:
  - Import Better Auth client library hooks (`useSession`, `useAuth`)
  - Modify `AuthProvider` to wrap with Better Auth provider
  - Export `useAuthContext` hook that accesses Better Auth state
  - Sync user data (email, name, profile) with AuthContext state
  - Add session persistence logic (check session on app mount)

- [ ] T018 Create `website/src/lib/authClient.ts`:
  - Initialize Better Auth client with API base URL
  - Configure client to use `/api/auth/*` endpoints
  - Export auth client instance for use in components

- [ ] T019 Create `website/src/hooks/useAuthContext.ts`:
  - Wrapper hook that provides `useAuth()` from Better Auth
  - Returns: `user`, `session`, `isLoading`, `login()`, `logout()`, `signup()`
  - Handles loading states and error catching

- [ ] T020 Update `website/src/services/apiClient.ts`:
  - Modify axios instance to include auth token in request headers
  - Add interceptor to get token from AuthContext and attach as `Authorization: Bearer <token>`
  - Handle 401 responses by redirecting to sign-in

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - User Registration with Background Questions (Priority: P1) 🎯 MVP

**Goal**: New users can create an account through a multi-step sign-up form that collects email/password and background information

**Independent Test**: Can verify by completing sign-up flow and checking user exists in database with all fields populated

### Tests for User Story 1

- [ ] T021 [P] [US1] Contract test for signup endpoint in `backend/tests/auth/test_signup_contract.py`:
  - Test POST `/api/auth/signup` with valid data returns 200 with user and token
  - Test returns 400 with invalid email format
  - Test returns 400 with weak password (< 8 chars)
  - Test returns 409 when email already exists
  - Test returns 500 on database error (mock DB failure)

- [ ] T022 [P] [US1] Integration test for signup flow in `backend/tests/auth/test_signup_integration.py`:
  - Test full signup: create user, verify in database, verify password hashed, verify profile fields saved
  - Test with minimal data, test with all profile fields
  - Test concurrent signup attempts (same email) - one succeeds, one fails with 409

- [ ] T023 [P] [US1] Frontend component test for SignUpForm in `website/tests/auth/signup.test.tsx`:
  - Test step 1: email/password form renders
  - Test validation: weak password rejected, invalid email rejected
  - Test advancing to step 2 with valid input
  - Test step 2: profile questions render (programming backgrounds, frameworks, hardware, robotics, experience level)
  - Test form submission calls auth API with all data

### Implementation for User Story 1

#### Backend Implementation

- [ ] T024 [US1] Implement `POST /api/auth/signup` endpoint in `backend/api/routes/auth.py`:
  - Handler function `signup(signup_request: SignUpRequest, db: Session)` -> AuthResponse
  - Validate request: email format, password strength (8+ chars), required fields
  - Call `user_service.create_user()` to create user and profile
  - Call `auth_service.generate_token()` to create JWT
  - Return AuthResponse with user and token
  - Error handling: return 400 for validation, 409 for duplicate email, 500 for DB errors

- [ ] T025 [US1] Create helper function `validate_password_strength(password: str) -> tuple[bool, str]` in `backend/api/services/auth_service.py`:
  - Validate length >= 8
  - Return (is_valid, error_message)

- [ ] T026 [US1] Implement `user_service.create_user()` in `backend/api/services/user_service.py`:
  - Hash password using `auth_service.hash_password()`
  - Create user record (email, password_hash, name)
  - Create user_profile record (programming_backgrounds, frameworks_known, hardware_experience, robotics_interest, experience_level, completed_onboarding=true)
  - Use database transaction (commit on success, rollback on error)
  - Return User object with profile fields populated

- [ ] T027 [US1] Implement `user_service.get_user_by_email()` in `backend/api/services/user_service.py`:
  - Query users table by email
  - Join user_profiles table
  - Return User with profile or None

#### Frontend Implementation

- [ ] T028 [US1] Create `website/src/components/auth/SignUpForm.tsx` component:
  - Multi-step form using react-hook-form
  - Step 1: email (type: email), password (type: password), confirm password, name (text)
    - Validation: email format, password >= 8 chars, password === confirm password, name required
    - Button: "Next" (advances to step 2)
  - Step 2: programming_backgrounds (checkboxes), frameworks_known (checkboxes), hardware_experience (checkboxes), robotics_interest (select), experience_level (select)
    - Validation: at least one selection for each required field
    - Button: "Create Account" (submits form)
  - Form state: `step` (1 or 2), `formData` (aggregates both steps)
  - Submit handler calls `POST /api/auth/signup` with all data
  - Error display: show error messages below relevant fields
  - Loading state: disable button while submitting

- [ ] T029 [US1] Create `website/src/pages/signup.tsx` page:
  - Import SignUpForm component
  - On successful signup: call `authContext.login()` and redirect to home page
  - Display success message or toast notification
  - Handle errors: display error modal/toast to user
  - On mount: check if user already logged in, redirect to home if so

- [ ] T030 [US1] Create `website/src/components/auth/BackgroundQuestions.tsx` sub-component:
  - Renders step 2 form fields (programming languages, frameworks, hardware, robotics, experience)
  - Reusable for signup and profile edit pages
  - Maps data to/from form values

- [ ] T031 [US1] Update `website/src/theme/Layout/index.tsx` navbar:
  - Add link to `/signup` page
  - Display "Sign Up" button (visible when not logged in)
  - Export navbar component so it can include auth buttons

### Validation & Error Handling

- [ ] T032 [US1] Add password validation function in `website/src/lib/auth-utils.ts`:
  - `validatePassword(password: string): {valid: boolean, errors: string[]}`
  - Check length >= 8, suggest if too weak
  - Call from SignUpForm

- [ ] T033 [US1] Add email validation in both backend and frontend:
  - Backend: `re.match(r'^[^\s@]+@[^\s@]+\.[^\s@]+$', email)`
  - Frontend: HTML5 email input + react-hook-form validation

**Checkpoint**: User Story 1 (signup) is complete and testable independently

---

## Phase 4: User Story 2 - User Login with Session Management (Priority: P1)

**Goal**: Existing users can log in and maintain a secure session across page refresh

**Independent Test**: Can verify by logging in, refreshing page, and confirming still logged in

### Tests for User Story 2

- [ ] T034 [P] [US2] Contract test for signin endpoint in `backend/tests/auth/test_signin_contract.py`:
  - Test POST `/api/auth/signin` with valid credentials returns 200 with user and token
  - Test returns 401 with wrong password
  - Test returns 401 with non-existent email (same message as wrong password to prevent email enumeration)
  - Test token is valid JWT and expires in 24 hours

- [ ] T035 [P] [US2] Integration test for signin flow in `backend/tests/auth/test_signin_integration.py`:
  - Test full signin: user logged in, token valid, user profile loaded
  - Test with wrong password: returns 401
  - Test case insensitivity on email (or not - decide and document)
  - Test session creation if using server-side sessions

- [ ] T036 [P] [US2] Contract test for session endpoint in `backend/tests/auth/test_session_contract.py`:
  - Test GET `/api/auth/session` with valid token returns 200 with current user
  - Test returns 401 without token
  - Test returns 401 with expired token

- [ ] T037 [P] [US2] Frontend component test for SignInForm in `website/tests/auth/signin.test.tsx`:
  - Test email and password fields render
  - Test form submission calls auth API
  - Test success redirects to home
  - Test error displays error message
  - Test "Forgot Password?" link (optional - can be out of scope for MVP)

- [ ] T038 [P] [US2] Integration test for session persistence in `website/tests/auth/session-persistence.test.tsx`:
  - Test login, then refresh page, verify user still logged in
  - Test JWT token persisted in localStorage or cookie
  - Test logout clears token

### Implementation for User Story 2

#### Backend Implementation

- [ ] T039 [US2] Implement `POST /api/auth/signin` endpoint in `backend/api/routes/auth.py`:
  - Handler function `signin(signin_request: SignInRequest, db: Session)` -> AuthResponse
  - Call `user_service.get_user_by_email(email)` -> User or None
  - If not found, return 401 "Invalid email or password" (no email enumeration)
  - If found, call `auth_service.verify_password(provided_password, user.password_hash)` -> bool
  - If password doesn't match, return 401 "Invalid email or password"
  - If valid, call `auth_service.generate_token(user.id)` to create JWT
  - Return AuthResponse with user and token

- [ ] T040 [US2] Implement `GET /api/auth/session` endpoint in `backend/api/routes/auth.py`:
  - Handler function `get_session(current_user: User = Depends(get_current_user), db: Session)` -> UserResponse
  - Middleware/dependency `get_current_user()` extracts and validates JWT token from header
  - Return current user with full profile
  - Raises 401 if no valid token

- [ ] T041 [US2] Implement token refresh (optional for MVP) in `backend/api/services/auth_service.py`:
  - `refresh_token(expired_token: str) -> tuple[str, datetime]` function
  - Validates that token is only slightly expired (within refresh window, e.g., 5 minutes)
  - Issues new token with fresh expiration
  - Returns 401 if token too old to refresh

#### Frontend Implementation

- [ ] T042 [US2] Update `website/src/pages/signin.tsx` page:
  - Import SignInForm component
  - On successful signin: call `authContext.login()` with token, set AuthContext state
  - Redirect to home page or referrer page
  - Display error message on login failure
  - Check if already logged in on mount, redirect to home if so

- [ ] T043 [US2] Create `website/src/components/auth/SignInForm.tsx` component:
  - Email field (type: email), password field (type: password)
  - Validation: email required, password required
  - Submit button: "Sign In"
  - Form submission calls `POST /api/auth/signin`
  - Error display: show error message below form
  - Loading state: disable button while submitting
  - Optional: "Forgot Password?" link (mark as out-of-scope for MVP)
  - Optional: "Sign Up" link (redirect to `/signup`)

- [ ] T044 [US2] Implement session persistence in `website/src/context/AuthContext.tsx`:
  - On AuthProvider mount: call `GET /api/auth/session` to check if user already logged in
  - If valid token in localStorage/cookie, restore user state
  - Set loading state while checking session
  - Handle session expiration: return 401 → clear token → redirect to signin

- [ ] T045 [US2] Implement `apiClient` interceptor in `website/src/services/apiClient.ts`:
  - Add auth token to every request: `Authorization: Bearer <token>`
  - On 401 response: redirect to `/signin`
  - Optional: refresh token automatically if token near expiration

- [ ] T046 [US2] Update `website/src/components/ProtectedRoute.tsx`:
  - Check `useAuthContext().isLoading` → show loading spinner
  - Check `useAuthContext().user` → if null, redirect to signin
  - Otherwise render children
  - Add experience level check if needed (though all logged-in users should have access to MVP features)

### Session Security

- [ ] T047 [US2] Configure token storage:
  - Decision: HTTP-only cookies vs. localStorage
  - If localStorage: implement with awareness of XSS risk (use react-hook-form/axios for API)
  - If cookies: ensure Secure and SameSite flags set in response headers

- [ ] T048 [US2] Add CORS headers for auth requests in `backend/api/app.py`:
  - Allow credentials: `Access-Control-Allow-Credentials: true`
  - Whitelist allowed origins for auth requests (not wildcard)

**Checkpoint**: User Stories 1 & 2 (signup and signin with session persistence) are complete and testable independently

---

## Phase 5: User Story 3 - User Logout and Session Invalidation (Priority: P1)

**Goal**: Logged-in users can click logout to end their session and be unable to access protected features

**Independent Test**: Can verify by logging in, clicking logout, and confirming redirect + protected page access fails

### Tests for User Story 3

- [ ] T049 [P] [US3] Contract test for signout endpoint in `backend/tests/auth/test_signout_contract.py`:
  - Test POST `/api/auth/signout` with valid token returns 200 with success message
  - Test POST `/api/auth/signout` without token still returns 200 (graceful)
  - Test that previous token is invalidated (GET `/api/auth/session` returns 401)

- [ ] T050 [P] [US3] Integration test for logout in `website/tests/auth/logout.test.tsx`:
  - Test click logout button while logged in
  - Test token is cleared from storage
  - Test redirect to home or signin page
  - Test accessing protected page after logout redirects to signin

### Implementation for User Story 3

#### Backend Implementation

- [ ] T051 [US3] Implement `POST /api/auth/signout` endpoint in `backend/api/routes/auth.py`:
  - Handler function `signout(current_user: User = Depends(get_current_user), db: Session)` -> SuccessResponse
  - Optional: add token to blacklist in database (if using server-side invalidation)
  - Optional: delete session record if using sessions table
  - Return `{"success": true}` or similar success response
  - Graceful if current_user is None (no token) - still return 200

#### Frontend Implementation

- [ ] T052 [US3] Create logout button in `website/src/components/auth/AuthHeader.tsx` component:
  - Display user's name and dropdown menu (if logged in)
  - Dropdown menu: "Profile" link, "Logout" button
  - Logout button calls `authContext.logout()` function
  - On logout: clear auth token, update AuthContext state
  - Redirect to home page or signin page
  - Show logout confirmation toast/notification

- [ ] T053 [US3] Update navbar/header in `website/src/theme/Layout/index.tsx`:
  - Import and display AuthHeader component
  - Show "Sign In" / "Sign Up" buttons when not logged in
  - Show AuthHeader when logged in
  - Handle responsive/mobile nav if needed

- [ ] T054 [US3] Implement `logout()` function in `website/src/context/AuthContext.tsx`:
  - Call `POST /api/auth/signout` endpoint
  - Clear auth token from storage (localStorage/cookies)
  - Set AuthContext state to null (clear user, session)
  - Redirect to home page

- [ ] T055 [US3] Add logout handling in `website/src/services/apiClient.ts`:
  - On 401 response (or token expired): automatically call logout
  - Clear token from storage
  - Redirect to signin

**Checkpoint**: User Stories 1, 2, & 3 (full authentication flow) are complete and testable independently

---

## Phase 6: User Story 4 - View and Update User Profile (Priority: P2)

**Goal**: Logged-in users can view and edit their profile information

**Independent Test**: Can verify by navigating to profile page, editing fields, saving, and confirming changes persisted

### Tests for User Story 4

- [ ] T056 [P] [US4] Contract test for profile endpoints in `backend/tests/auth/test_profile_contract.py`:
  - Test GET `/api/users/profile` returns 200 with current user's profile
  - Test PUT `/api/users/profile` with valid data returns 200 with updated profile
  - Test PUT `/api/users/profile` with invalid email returns 400
  - Test returns 401 without auth token

- [ ] T057 [P] [US4] Integration test for profile updates in `backend/tests/auth/test_profile_integration.py`:
  - Test updating each field individually
  - Test updating multiple fields at once
  - Test verify updates persisted in database
  - Test concurrent profile updates (second request overrides first)

- [ ] T058 [P] [US4] Frontend component test for ProfilePage in `website/tests/auth/profile.test.tsx`:
  - Test profile page renders current user info
  - Test edit fields are editable
  - Test save button calls API
  - Test success message after save
  - Test redirect to signin if not logged in

### Implementation for User Story 4

#### Backend Implementation

- [ ] T059 [US4] Implement `GET /api/users/profile` endpoint in `backend/api/routes/users.py`:
  - Handler function `get_profile(current_user: User = Depends(get_current_user), db: Session)` -> UserResponse
  - Return current user with all profile fields

- [ ] T060 [US4] Implement `PUT /api/users/profile` endpoint in `backend/api/routes/users.py`:
  - Handler function `update_profile(current_user: User = Depends(get_current_user), profile_request: ProfileUpdateRequest, db: Session)` -> UserResponse
  - Call `user_service.update_user_profile(current_user.id, profile_request.dict())`
  - Return updated user with all profile fields
  - Error handling: 400 for validation errors, 500 for DB errors

- [ ] T061 [US4] Implement `user_service.update_user_profile()` in `backend/api/services/user_service.py`:
  - Accept user_id and profile_data dict
  - Update user_profiles table with provided fields
  - Only update provided fields (allow partial updates)
  - Return updated User object
  - Handle database transaction

#### Frontend Implementation

- [ ] T062 [US4] Create `website/src/pages/profile.tsx` page:
  - On mount: verify user is logged in (redirect to signin if not)
  - Call `GET /api/users/profile` to fetch current profile
  - Display loading state while fetching
  - Render ProfileForm component with fetched data

- [ ] T063 [US4] Create `website/src/components/auth/ProfileForm.tsx` component:
  - Reuse BackgroundQuestions sub-component for profile fields
  - Add name field (editable text)
  - Add email field (read-only or editable with verification)
  - Add "Save Profile" button
  - Form submission calls `PUT /api/users/profile` with updated fields
  - Success: show toast notification, update local state
  - Error: show error message in form

- [ ] T064 [US4] Add profile link to AuthHeader dropdown (in T052):
  - "Profile" link routes to `/profile`

**Checkpoint**: User Stories 1-4 (authentication and profile management) are complete and testable independently

---

## Phase 7: User Story 5 - Chatbot Aware of User Profile (Priority: P2)

**Goal**: The RAG chatbot has access to logged-in user's profile and tailors responses by expertise level

**Independent Test**: Can verify by logging in as different expertise levels and checking chatbot response differences

### Tests for User Story 5

- [ ] T065 [P] [US5] Contract test for chat with user context in `backend/tests/chat/test_chat_with_user_context.py`:
  - Test POST `/api/chat` with user object in request returns 200
  - Test user context is passed to OpenRouter API in system prompt
  - Test response quality differs based on experience_level (beginner vs. advanced)
  - Test anonymous user (no user field) gets generic response

- [ ] T066 [P] [US5] Integration test for personalized chatbot in `backend/tests/chat/test_personalized_responses.py`:
  - Test ask same question as beginner user, get simplified response
  - Test ask same question as advanced user, get technical response
  - Verify chatbot service receives user profile in context

- [ ] T067 [P] [US5] Frontend integration test for chatbot with auth in `website/tests/chat/chatbot-with-auth.test.tsx`:
  - Test logged-in user's profile is sent with chat request
  - Test anonymous user can still chat (no profile sent)

### Implementation for User Story 5

#### Backend Implementation

- [ ] T068 [US5] Update `backend/api/models.py` Chat/ChatRequest models:
  - Add optional `user` field to ChatRequest: `user: Optional[dict] = None`
  - Structure: `{ "id": "...", "experience_level": "...", "programming_backgrounds": [...], ... }`

- [ ] T069 [US5] Update `backend/api/routes/chat.py` endpoints:
  - Modify request body to include optional user field
  - Pass user context to `chat_service.generate_response()`
  - No change to response format (response is same whether user context present or not)

- [ ] T070 [US5] Update chatbot service in `backend/api/services/` (or equivalent):
  - Modify to accept optional `user` parameter: `generate_response(query: str, user: Optional[dict] = None)`
  - Build system prompt that includes user context if present:
    ```
    User's experience level: {{user.experience_level}}
    User's hardware experience: {{user.hardware_experience}}
    Adjust response complexity accordingly.
    ```
  - If no user, use generic system prompt
  - Pass modified system prompt to OpenRouter API

- [ ] T071 [US5] Document response tailoring in chatbot logic:
  - Beginner: use simple language, explain concepts, avoid jargon
  - Intermediate: assume some knowledge, can use some technical terms
  - Advanced: can use specialized terminology, assume prior knowledge
  - This may be implemented in system prompt or in response post-processing

#### Frontend Implementation

- [ ] T072 [US5] Update ChatBot component in `website/src/components/ChatBot.tsx`:
  - Get current user from `useAuthContext()`
  - Extract user profile: `{ experience_level, hardware_experience, ... }`
  - Pass user object in chat request to backend:
    ```typescript
    const response = await apiClient.post('/api/chat', {
      query: userMessage,
      user: currentUser ? {
        id: currentUser.id,
        experience_level: currentUser.experience_level,
        hardware_experience: currentUser.hardware_experience,
        ...
      } : undefined
    })
    ```
  - If not logged in, omit user field (undefined is handled by backend)

- [ ] T073 [US5] Add visual indicator in ChatBot UI (optional enhancement):
  - Show message like "Personalizing responses for {{experience_level}} users" or similar
  - Shows user chatbot is aware of their profile
  - Hide for anonymous users

**Checkpoint**: User Stories 1-5 (authentication and personalized chatbot) are complete and testable independently

---

## Phase 8: User Story 6 - Persistent Login State in Header/Navbar (Priority: P2)

**Goal**: Header displays login state clearly; shows Sign In/Sign Up for anonymous, user name and logout for authenticated

**Independent Test**: Can verify by checking navbar displays correct state before/after login

### Implementation for User Story 6

- [ ] T074 [US6] Create `website/src/components/auth/AuthHeader.tsx` component (if not already created in T052):
  - Anonymous state: "Sign In" and "Sign Up" buttons
  - Authenticated state: User name + dropdown menu (Profile link, Logout button)
  - Loading state: show skeleton or spinner while checking auth status

- [ ] T075 [US6] Update navbar in `website/src/theme/Layout/index.tsx` (if not already done):
  - Import AuthHeader
  - Display AuthHeader in navbar
  - Ensure responsive design (hamburger menu on mobile if needed)

- [ ] T076 [US6] Add styling for auth buttons:
  - Primary color for Sign Up button
  - Secondary color for Sign In button
  - Dropdown menu styling for logged-in state
  - Match Docusaurus theme colors

- [ ] T077 [US6] Add smooth transitions:
  - Animate between logged-in and logged-out states
  - No jarring layout shifts when auth state changes
  - Loading spinner while session is being verified

**Checkpoint**: User Stories 1-6 (all user stories) are complete and testable independently

---

## Phase 9: Cross-Cutting Concerns & Polish

**Purpose**: Improvements that affect multiple user stories, security, testing, and documentation

### Security Hardening

- [ ] T078 [P] Add password strength indicator in frontend SignUp form:
  - Show password requirements: minimum 8 characters
  - Visual indicator of password strength as user types (optional)
  - Prevent submit if password too weak

- [ ] T079 [P] Implement HTTPS/SSL requirement in production:
  - Verify backend returns Secure flag on cookies
  - Verify frontend redirects HTTP to HTTPS
  - Document in deployment guide

- [ ] T080 [P] Add rate limiting on auth endpoints (post-MVP consideration):
  - Document in plan: "To be implemented in Phase 4"
  - Example: 5 signin attempts per IP per minute

### Error Handling & User Feedback

- [ ] T081 [P] Standardize error responses in backend:
  - All auth endpoints return consistent error format: `{ "error": "message", "code": "CODE" }`
  - Document error codes (e.g., "INVALID_CREDENTIALS", "USER_EXISTS", "VALIDATION_ERROR")
  - No server stack traces exposed to client

- [ ] T082 [P] Add user-friendly error messages in frontend:
  - "Invalid email or password" for login failures
  - "This email is already in use" for signup conflicts
  - "Password must be at least 8 characters" for weak passwords
  - Display validation errors next to form fields, not in modals

- [ ] T083 [P] Add loading states and spinners:
  - Show spinner during signup, signin, profile save
  - Disable buttons while request is in flight
  - Cancel button (optional) to abort pending requests

### Logging & Monitoring

- [ ] T084 [P] Add logging for auth events in backend:
  - Log signup: `"User signup attempt: email={email}"`
  - Log successful signin: `"User signed in: user_id={user_id}"`
  - Log failed signin: `"Failed signin attempt: email={email}"`
  - Log logout: `"User signed out: user_id={user_id}"`
  - Use appropriate log levels (INFO for success, WARNING for failed attempts)

- [ ] T085 [P] Add error tracking in frontend:
  - Log signup/signin errors to console in dev
  - Send errors to monitoring service (e.g., Sentry) in production
  - Track failed auth attempts for alerting

### Testing & Validation

- [ ] T086 Run all auth tests from Phase 3-8:
  - Backend tests: `pytest backend/tests/auth/ -v`
  - Frontend tests: `npm test website/tests/auth/ --watch=false`
  - Verify all tests pass

- [ ] T087 Run E2E test for full signup → login → chatbot flow:
  - Create Playwright test in `website/tests/e2e/auth-full-flow.spec.ts`
  - Scenario: User signs up, logs out, logs in, uses chatbot, checks profile
  - Verify all steps work end-to-end

- [ ] T088 Validate schema compliance:
  - Verify database schema matches design doc
  - Verify all tables created and indexes applied
  - Run schema validation script if available

- [ ] T089 Manual testing checklist:
  - [ ] Signup with all fields
  - [ ] Signup with minimal fields
  - [ ] Signup with invalid email → error
  - [ ] Signup with weak password → error
  - [ ] Signup with existing email → error
  - [ ] Signin with correct credentials → success
  - [ ] Signin with wrong password → error
  - [ ] Signin, refresh page → still logged in
  - [ ] Click logout → redirect to home, cannot access protected page
  - [ ] Edit profile → changes saved
  - [ ] Logged in user asks chatbot question → response is personalized
  - [ ] Anonymous user asks chatbot question → response is generic

### Documentation

- [ ] T090 Create `specs/003-better-auth-integration/quickstart.md`:
  - How to run auth locally (both backend and frontend)
  - Environment variables needed
  - Database setup steps
  - Example requests/responses for auth endpoints
  - Troubleshooting guide

- [ ] T091 Create `specs/003-better-auth-integration/contracts/auth-endpoints.md`:
  - POST `/api/auth/signup`: request/response examples
  - POST `/api/auth/signin`: request/response examples
  - POST `/api/auth/signout`: request/response
  - GET `/api/auth/session`: request/response
  - Error responses

- [ ] T092 Create `specs/003-better-auth-integration/contracts/user-profile-endpoints.md`:
  - GET `/api/users/profile`: request/response
  - PUT `/api/users/profile`: request/response
  - Error responses

- [ ] T093 Create `specs/003-better-auth-integration/contracts/session-management.md`:
  - JWT token structure (claims, expiration)
  - Session storage (localStorage vs cookies)
  - Token refresh mechanism (if implemented)
  - Logout and token invalidation

- [ ] T094 Update main README.md:
  - Add section on authentication
  - Link to quickstart.md
  - Link to API contracts

### Code Quality

- [ ] T095 [P] Code review all auth code:
  - Security: no hardcoded secrets, proper validation
  - Consistency: naming conventions, error handling patterns
  - Testing: all critical paths covered
  - Documentation: functions have docstrings/comments

- [ ] T096 [P] Run linting and format checks:
  - Backend: `black`, `flake8`, or similar
  - Frontend: `prettier`, `eslint`
  - Fix any warnings

### Optional Enhancements (Document as Future Work)

- [ ] T097 Email verification flow (mark as out-of-scope for MVP)
- [ ] T098 Password reset / forgot password flow (mark as out-of-scope)
- [ ] T099 Two-factor authentication (2FA) (mark as out-of-scope)
- [ ] T100 Social login (Google, GitHub) (mark as out-of-scope)
- [ ] T101 Role-based access control (RBAC) beyond authenticated/anonymous (mark as out-of-scope)

---

## Phase 10: Final Validation & Launch Readiness

**Purpose**: Verify feature is production-ready before launch

- [ ] T102 Security audit:
  - Verify no secrets in code
  - Verify password hashing working correctly
  - Verify JWT tokens generated and validated properly
  - Check for SQL injection vulnerabilities (if using raw SQL)
  - Check for XSS vulnerabilities (sanitize user input)

- [ ] T103 Performance testing:
  - Measure signup response time: should be < 1 second (excluding network)
  - Measure signin response time: should be < 500ms
  - Test with 100 concurrent signups: no timeouts
  - Test with 100 concurrent signins: no timeouts

- [ ] T104 Database validation:
  - Verify constraints (unique email, foreign keys, NOT NULLs)
  - Verify indexes exist (email, user_id)
  - Run integrity check query
  - Test backup/restore procedure

- [ ] T105 Deployment testing on staging environment:
  - Deploy frontend to staging
  - Deploy backend to staging
  - Run full signup → login → chatbot → logout flow on staging
  - Verify all features work in staging

- [ ] T106 Create deployment runbook:
  - Steps to deploy backend (migrations, env vars, restart service)
  - Steps to deploy frontend (build, upload to CDN/hosting)
  - Steps to rollback if needed
  - Monitoring/alerting setup
  - On-call procedures for auth incidents

- [ ] T107 Create operational monitoring:
  - Monitor auth endpoint latencies
  - Alert on high error rates (>1% failures)
  - Track user signup/signin metrics
  - Set up log aggregation and search

---

## Dependencies & Execution Order

### Phase Dependencies

1. **Setup (Phase 1)**: No dependencies - start here
2. **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
3. **User Stories (Phases 3-8)**: All depend on Foundational
   - US1, US2, US3 can proceed sequentially or in parallel (all are P1)
   - US4, US5, US6 can proceed in parallel after US1 complete
4. **Cross-Cutting (Phase 9)**: After all user stories, no dependencies between tasks
5. **Final Validation (Phase 10)**: After all code complete and tested

### Critical Path

```
T001-T005 (Setup)
    ↓
T006-T020 (Foundational)
    ↓
T021-T032 (US1: Signup tests + implementation)
    ↓
T034-T048 (US2: Signin tests + implementation)
    ↓
T049-T055 (US3: Logout tests + implementation)
    ↓
T056-T076 (US4, US5, US6: Profile, Chatbot, Navbar - can parallelize)
    ↓
T078-T094 (Cross-cutting concerns: Security, logging, docs)
    ↓
T102-T107 (Final validation & launch)
```

### Parallel Opportunities

- **Setup Phase**: All tasks marked [P] can run in parallel
- **Foundational Phase**: Many tasks marked [P] can run in parallel (different files):
  - T006 (DB migration) can start immediately
  - T008-T010 (Pydantic schemas) can start immediately
  - T011-T013 (Auth services) can start once schemas done
  - T017-T020 (Frontend infra) independent of backend
- **Once Foundational is done**: All user story tests (T021, T034, T049, etc.) can run in parallel
- **Within each user story**: Tests can run in parallel while implementation happens
- **Cross-Cutting Phase**: All tasks marked [P] can run in parallel

### Recommended Team Structure

**Small team (1-2 developers)**:
- Sequential: Setup → Foundational → US1 → US2 → US3 → US4/US5/US6 in parallel → Polish → Launch

**Medium team (3-4 developers)**:
- Setup together (1-2 days)
- Foundational in parallel: Backend services + Frontend infra
- Once foundational done: Backend dev does US1 auth routes, Frontend dev does US1 forms
- Then: Backend does US2-3 (signin/logout), Frontend does US2-3, then parallelize US4-6
- Polish and launch together

**Large team (5+ developers)**:
- Foundational work split between backend and frontend developers
- Parallelize US1-3 (different devs on backend vs frontend)
- Parallelize US4-6 fully
- Cross-cutting and deployment in parallel with user story work

---

## Implementation Strategy

### MVP First (User Stories 1-3 Only)

**Target**: Basic auth working (signup, signin, logout)

1. Complete T001-T020 (Setup + Foundational) - 2-3 days
2. Complete T021-T048 (US1 + US2) - 3-4 days
3. Complete T049-T055 (US3) - 1-2 days
4. Run T086-T089 (Testing & validation) - 1 day
5. **STOP and VALIDATE**: Test full signup → login → logout flow
6. If working, can deploy early (MVP is done)
7. Then proceed to US4-6

### Incremental Delivery

1. **MVP (Day 1-7)**: US1-3 (auth working)
2. **Phase 2 (Day 8-9)**: US4 (profile management)
3. **Phase 3 (Day 10-11)**: US5 (chatbot personalization)
4. **Phase 4 (Day 12)**: US6 (header UI polish)
5. **Launch Day**: T102-T107 (final validation) and go live

Each phase delivers value independently and can be demoed/deployed alone.

---

## Checkpoints for Sign-Off

After each phase, verify:

- [ ] **After Foundational (T020)**: Database schema created, migrations run, all services initialized, no compilation errors
- [ ] **After US1 (T032)**: User can sign up, account created in database with all fields, tests passing
- [ ] **After US2 (T048)**: User can log in, session persists across refresh, tests passing
- [ ] **After US3 (T055)**: User can logout, session invalidated, cannot access protected pages
- [ ] **After US4 (T064)**: User can view/edit profile, changes saved to database
- [ ] **After US5 (T073)**: Chatbot receives user profile and tailors responses
- [ ] **After US6 (T077)**: Header displays auth state correctly (signed in/out)
- [ ] **After Cross-Cutting (T094)**: All docs written, tests passing, no security issues
- [ ] **After Final Validation (T107)**: Staging deployment working, monitoring in place, ready for production

---

## Notes & Best Practices

- **[P] tasks** = can run in parallel (different files, no dependencies)
- **[Story] label** = maps task to specific user story for traceability
- **Each user story independently completable** = can be tested alone without other stories
- **Write tests first**: Verify they fail before implementing (TDD approach recommended)
- **Commit after each task or logical group**: Keep git history clean and reviewable
- **Stop at any checkpoint**: Can demo/validate/deploy at US1, US2, US3, US4, US5, US6 independently
- **Avoid**: Vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Success Criteria (from Spec)

✅ SC-001: Users can complete sign-up in under 2 minutes
✅ SC-002: Users can log in successfully in under 1 minute
✅ SC-003: 100% of user passwords are hashed before storage
✅ SC-004: Session tokens persist across page refresh with 100% reliability
✅ SC-005: Logged-in users cannot access protected endpoints after logout
✅ SC-006: System handles 100 concurrent sign-ups without timeouts
✅ SC-007: Profile updates reflected in chatbot responses within 2 seconds
✅ SC-008: Auth integration doesn't increase page load time by >500ms
✅ SC-009: All auth endpoints return consistent, clear error messages

**Tasks Status**: 🟢 Ready for implementation

**Next Step**: Begin Phase 1 (Setup) and Phase 2 (Foundational) when assigned to developer
