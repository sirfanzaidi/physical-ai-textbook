# Implementation Plan: Better Auth Integration & User Authentication

**Branch**: `003-better-auth-integration` | **Date**: 2025-12-18 | **Spec**: `specs/003-better-auth-integration/spec.md`

**Input**: Feature specification from `/specs/003-better-auth-integration/spec.md`

## Summary

Integrate secure user authentication into the Docusaurus-based Physical AI textbook using Better Auth on the frontend and a FastAPI-based backend with PostgreSQL persistence. Users will register via a multi-step form capturing background information, log in securely, and the floating RAG chatbot will be aware of user profiles to tailor responses by expertise level. The implementation leverages existing scaffolding (SignIn/SignUp pages, AuthContext, ProtectedRoute) and extends the backend with user/session management endpoints.

---

## Technical Context

**Language/Version**: Python 3.11 (backend), TypeScript 5 / React 19 (frontend)

**Primary Dependencies**:
- Backend: FastAPI, SQLAlchemy (or psycopg2 for direct queries), passlib[bcrypt], python-jose
- Frontend: better-auth@^1.4.5, react-hook-form, axios, React 19

**Storage**: PostgreSQL (Neon) for users, sessions, profiles; Qdrant for vector embeddings (RAG, not auth)

**Testing**: pytest (backend integration/unit), Vitest/Jest (frontend), Playwright (E2E)

**Target Platform**: Web (Docusaurus site + FastAPI API)

**Project Type**: Web (frontend + backend separation)

**Performance Goals**:
- Sign-up completion: < 2 minutes
- Login completion: < 1 minute
- Auth endpoint latency (p95): < 500ms
- Session persistence: 100% reliability across refresh

**Constraints**:
- Minimum password: 8 characters
- No destructive changes to existing RAG chatbot or Docusaurus site structure
- CORS must remain configured; new auth endpoints must be included
- Database connection must use existing Neon pool (no new DB systems)

**Scale/Scope**:
- Initial: 100s of users (during MVP testing)
- Projected: 10,000+ concurrent authenticated users

---

## Constitution Check

**Gate: Feature Specification Review**

✅ **Passes**:
- Clear functional requirements (FR-001 through FR-015)
- Well-defined user stories with acceptance criteria
- Security measures specified (password hashing, session tokens)
- Non-functional requirements stated (performance, availability, security)
- Known constraints and out-of-scope items documented

**Flag for Architecture Review**: Better Auth backend integration approach needs clarification
- Current spec assumes FastAPI-based auth backend OR Better Auth server adapter
- During Phase 1 design, must decide:
  - Option A: Use Better Auth's JavaScript client + custom FastAPI backend (most flexibility)
  - Option B: Use Better Auth's Node.js server adapter in separate service (standard approach)
  - Option C: Use compatible alternative library on FastAPI side

*Will be resolved during Phase 1 (Design) before proceeding to Phase 2 (Implementation).*

---

## Project Structure

### Documentation (this feature)

```text
specs/003-better-auth-integration/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Phase 0 output (to be created during design phase)
├── data-model.md        # Phase 1 output - Database schema
├── contracts/           # Phase 1 output - API contracts
│   ├── auth-endpoints.md
│   ├── user-profile-endpoints.md
│   └── session-management.md
├── quickstart.md        # Phase 1 output - Setup instructions
└── tasks.md             # Phase 2 output - Task breakdown
```

### Source Code (repository root)

```text
# Backend Authentication Components
backend/
├── api/
│   ├── app.py                      # Enhanced: add auth routes middleware
│   ├── config.py                   # Enhanced: add auth settings (JWT_SECRET, etc.)
│   ├── models.py                   # NEW: User, UserProfile, Session models
│   ├── routes/
│   │   ├── auth.py                 # NEW: signup, signin, signout, session endpoints
│   │   ├── users.py                # NEW: profile endpoints (GET, PUT)
│   │   ├── chat.py                 # MODIFIED: add user context to requests
│   │   ├── ingest.py               # unchanged
│   │   └── health.py               # unchanged
│   ├── services/
│   │   ├── auth_service.py         # NEW: password hashing, token generation
│   │   ├── user_service.py         # NEW: user CRUD operations
│   │   ├── session_service.py      # NEW: session management
│   │   └── [existing RAG services] # unchanged
│   ├── middleware/
│   │   └── auth.py                 # NEW: JWT/session verification middleware
│   ├── database/
│   │   ├── postgres_client.py      # MODIFIED: add user/profile tables
│   │   ├── migrations/             # NEW: Alembic or manual migration scripts
│   │   │   └── 001_create_users_table.sql
│   │   └── [existing Qdrant client] # unchanged
│   └── schemas/                    # NEW: Pydantic request/response schemas
│       ├── auth.py
│       └── user.py
├── requirements.txt                # MODIFIED: add passlib, bcrypt, python-jose
├── main.py                         # unchanged
└── tests/
    └── auth/                       # NEW: auth-specific tests
        ├── test_signup.py
        ├── test_signin.py
        └── test_profile.py

# Frontend Authentication Components
website/
├── src/
│   ├── pages/
│   │   ├── signin.tsx              # MODIFIED: integrate Better Auth client
│   │   ├── signup.tsx              # MODIFIED: integrate Better Auth client, add step 2 logic
│   │   ├── profile.tsx             # MODIFIED: implement profile view/edit
│   │   └── [existing Docusaurus pages] # unchanged
│   ├── components/
│   │   ├── auth/                   # NEW: auth-specific components
│   │   │   ├── SignUpForm.tsx
│   │   │   ├── SignInForm.tsx
│   │   │   └── AuthHeader.tsx      # User state in navbar
│   │   ├── ProtectedRoute.tsx       # MODIFIED: use Better Auth context
│   │   ├── ChatBot.tsx             # MODIFIED: pass user profile to backend
│   │   └── [existing components]   # mostly unchanged
│   ├── context/
│   │   └── AuthContext.tsx         # MODIFIED: integrate Better Auth hooks
│   ├── lib/
│   │   ├── authClient.ts           # MODIFIED: configure Better Auth client
│   │   ├── apiConfig.ts            # unchanged
│   │   └── auth-utils.ts           # NEW: helper functions
│   ├── services/
│   │   ├── apiClient.ts            # MODIFIED: add auth headers to requests
│   │   └── [existing services]     # mostly unchanged
│   └── hooks/
│       └── useAuthContext.ts        # MODIFIED: Better Auth integration
├── package.json                    # MODIFIED: verify better-auth, add any missing deps
└── tests/
    └── auth/                       # NEW: auth-specific component tests
        ├── signin.test.tsx
        ├── signup.test.tsx
        └── profile.test.tsx
```

**Structure Decision**:
- Web application with clear backend/frontend separation
- Backend: FastAPI with modular service layer (auth_service, user_service, session_service)
- Frontend: Docusaurus + React with component-based organization
- Database: Single PostgreSQL (Neon) with migration management
- Existing RAG chatbot remains in the chat routes but receives user context as optional parameter

---

## Design Decisions & Rationale

### 1. Backend Architecture: FastAPI + SQLAlchemy/psycopg2

**Decision**: Implement auth backend directly in FastAPI (not a separate service)

**Rationale**:
- FastAPI already configured and deployed
- Adding auth endpoints avoids operational complexity of a second backend service
- PostgreSQL connection pool already established; reuse for user/session tables
- Keeps deployment simple: one backend service, one database

**Alternative Considered**: Separate Better Auth Node.js server
- ❌ Adds operational overhead, separate deployment pipeline
- ❌ Requires inter-service communication
- ✅ Would match Better Auth's standard architecture, but not necessary for MVP

**Reversibility**: If needed later, can extract auth service into separate service with minimal refactoring.

---

### 2. Session Management: JWT + HTTP-Only Cookies

**Decision**: Use JWT tokens stored in HTTP-only cookies (with CSRF protection if forms present)

**Rationale**:
- HTTP-only cookies prevent XSS attacks from stealing tokens
- JWT allows stateless verification (no need to query sessions table on every request)
- Familiar pattern for FastAPI developers
- Frontend already has localStorage pattern; easy to migrate to cookies

**Alternative Considered**: Server-side sessions (Redis + database)
- ❌ Adds complexity (Redis or database lookup on every request)
- ✅ More control over sessions; easier to revoke

**Implementation Detail**:
- Tokens include `exp` (expiration) and `user_id` claims
- Refresh token endpoint optional (can implement if needed post-MVP)
- Logout invalidates token (optional: add to blacklist if stateless validation insufficient)

---

### 3. Password Hashing: bcrypt

**Decision**: Use `passlib[bcrypt]` for password hashing

**Rationale**:
- Industry standard, slow by design (resistant to brute force)
- Works natively with FastAPI and Starlette
- Well-tested, audited library

**Configuration**:
- bcrypt cost factor: 12 (default, ~100ms per hash; balance between security and UX)

---

### 4. Frontend Auth State: Better Auth Client + AuthContext

**Decision**: Integrate Better Auth client library into existing AuthContext

**Rationale**:
- AuthContext already exists and is hooked up to ProtectedRoute and navbar
- Better Auth provides React hooks (`useSession`, `useAuth`) that integrate cleanly
- Minimal refactoring of existing code
- Frontend forms (SignIn/SignUp) already scaffolded; just add Better Auth calls

**Implementation**:
- `useAuthContext()` hook wraps `useSession()` from Better Auth
- AuthContext provider calls Better Auth client on mount to check session
- Existing components (ProtectedRoute, navbar) use AuthContext without change

---

### 5. Multi-Step Sign-Up Flow: Client-Side Validation

**Decision**: Multi-step form with client-side validation (react-hook-form) and backend validation

**Rationale**:
- Step 1 (email/password) validates before advancing to Step 2
- Step 2 (background) collects profile data
- Final submission sends all data to `/auth/signup` backend endpoint
- Backend re-validates all fields before creating user

**Alternative Considered**: Server-side step management
- ❌ Would require session/state tracking for partial form submissions
- ✅ Client-side is simpler for MVP; less backend complexity

---

### 6. Email Verification: Optional (Disabled in MVP)

**Decision**: Email verification is optional; disabled in MVP to reduce friction

**Rationale**:
- Reduces sign-up friction (users can start immediately)
- Can be added post-MVP without schema changes (add `email_verified` boolean column)
- For MVP, assume good-faith email addresses

**Re-evaluation**: Post-launch, if spam registrations become issue, enable email verification.

---

### 7. User Profile Awareness in Chatbot

**Decision**: Chatbot receives user profile (experience_level, hardware_background, etc.) in request; uses it to tailor responses

**Rationale**:
- Minimal changes to existing chatbot code: add optional `user` field to chat request
- Chatbot can check `user.experience_level` to adjust response complexity
- Non-authenticated users (anonymous) get generic responses
- Allows chatbot to remember user preferences without extra storage

**Implementation**:
- POST `/api/chat` and POST `/api/chat-stream` receive optional `user` object
- Chatbot system prompt can include user context: "User's experience level: {{experience_level}}"
- OpenRouter API call includes user context in system prompt

---

## Phase Breakdown

### Phase 0: Clarification & Research (Specification review)
**Status**: ✅ Complete (you are reviewing this now)

**Outputs**:
- ✅ Feature spec with user stories, requirements, success criteria
- ✅ Architecture decisions documented (this plan)
- ⏳ Backend framework choice clarified (FastAPI confirmed)
- ⏳ Better Auth integration approach finalized (JWT+cookies vs. alternative)

### Phase 1: Design (Before implementation)
**Status**: ⏳ Pending

**Deliverables**:
- Database schema (users, user_profiles, sessions tables)
- API contracts (request/response schemas, error handling, status codes)
- Frontend component specs (updated SignUp form steps, AuthContext modifications)
- Setup/quickstart guide (how to run locally, environment variables)

**Owner**: Architecture/Design team or lead engineer

### Phase 2: Implementation (Write code)
**Status**: ⏳ Pending

**Phase 2a: Backend**
1. Database schema & migrations
2. Pydantic models (User, UserProfile, Session)
3. Auth service (signup, signin, password hashing)
4. User service (CRUD, profile)
5. Auth routes (endpoints)
6. Auth middleware (JWT verification)
7. Integration with existing chat routes (pass user context)
8. Backend tests (unit + integration)

**Phase 2b: Frontend**
1. Enhanced AuthContext with Better Auth client
2. Updated SignUp component (multi-step)
3. Updated SignIn component
4. Profile page implementation
5. AuthHeader component (navbar user state)
6. ProtectedRoute updates (use Better Auth)
7. API client updates (pass auth headers/tokens)
8. Frontend tests

**Phase 2c: Integration**
1. E2E tests (signup → login → chatbot with profile)
2. Security review (password hashing, token generation)
3. Performance testing (auth endpoints latency)

**Owner**: Full-stack engineers (frontend + backend)

### Phase 3: Testing & Launch
**Status**: ⏳ Pending

**Activities**:
- Functional testing (all user stories)
- Security testing (OWASP top 10, auth vulnerabilities)
- Performance testing (concurrent users, response times)
- Deployment staging (test on staging environment before production)
- Monitoring setup (log auth events, alert on failures)

**Owner**: QA team + DevOps + Security

---

## Integration Points with Existing Features

### 1. RAG Chatbot (Existing)
- **Current State**: Works without authentication; responses are generic
- **Integration**: Modify `POST /api/chat` and `POST /api/chat-stream` to receive optional `user` field; chatbot uses `user.experience_level` to tailor responses
- **Impact**: Low; changes are backward compatible (user field is optional)
- **Risk**: If chatbot service has bugs, auth won't break it (auth is layer on top)

### 2. Docusaurus Site (Existing)
- **Current State**: No authentication UI; header has no login/logout
- **Integration**: Add AuthHeader component to navbar; add /signin and /signup pages (already exist, need integration)
- **Impact**: Minimal; mostly adding UI components to header
- **Risk**: CSS conflicts if navbar styling assumptions change

### 3. FastAPI Backend (Existing)
- **Current State**: Routes for chat, ingest, health; no auth
- **Integration**: Add new routes (auth, users); add middleware for JWT verification
- **Impact**: Moderate; adds new modules (auth_service, user_service) but doesn't modify existing services
- **Risk**: Database connection pool must be extended to support more concurrent connections (for user queries); monitor connection limits

### 4. PostgreSQL Database (Existing)
- **Current State**: Has chunks_metadata table; connection pool configured
- **Integration**: Add users, user_profiles, sessions tables; migration scripts
- **Impact**: New tables, no changes to existing schema
- **Risk**: If database is read-only, migrations will fail; need write access confirmed

---

## Key Dependencies & Risks

### Risk 1: Better Auth Compatibility with FastAPI Backend
**Severity**: High (impacts feasibility)
**Mitigation**:
- Phase 1 design must confirm Better Auth client works with FastAPI backend
- If compatibility issues, plan fallback: implement auth backend manually (more work, but feasible)
- No risk to existing features (auth is isolated)

### Risk 2: Database Connection Pool Exhaustion
**Severity**: Medium (impacts scalability)
**Mitigation**:
- Monitor connection usage during testing
- Increase pool size if needed (currently configured in Neon dashboard)
- Implement connection timeouts and retries

### Risk 3: Password Reset Flow Not Included in MVP
**Severity**: Low (affects post-launch usability, not MVP)
**Mitigation**:
- Document as out-of-scope for MVP
- Plan as Phase 4 feature post-launch
- Include forgotten password error message in frontend: "Contact support to reset password"

### Risk 4: Email Verification Disabled
**Severity**: Low (could allow spam accounts)
**Mitigation**:
- Monitor for suspicious registrations post-launch
- Add email verification in Phase 4 if needed
- Log all signup events for audit trail

### Risk 5: No Rate Limiting on Auth Endpoints
**Severity**: Medium (brute-force attacks possible)
**Mitigation**:
- Implement simple rate limiting post-MVP (e.g., 5 login attempts per IP per minute)
- Document as known limitation in launch notes

---

## Success Criteria (from Spec)

- ✅ SC-001: Users can complete sign-up in under 2 minutes
- ✅ SC-002: Users can log in successfully in under 1 minute
- ✅ SC-003: 100% of user passwords are hashed before storage
- ✅ SC-004: Session tokens persist across page refresh with 100% reliability
- ✅ SC-005: Logged-in users cannot access protected endpoints after logout
- ✅ SC-006: System handles 100 concurrent sign-ups without timeouts or data loss
- ✅ SC-007: Profile updates are reflected in the chatbot's user-aware responses within 2 seconds
- ✅ SC-008: Auth integration doesn't increase page load time by more than 500ms
- ✅ SC-009: All authentication endpoints return consistent, clear error messages

---

## Next Steps

1. **Phase 1 Design Review**: Stakeholder reviews this plan and approves approach
2. **Phase 1 Design Work**: Create database schema, API contracts, component specs
3. **Phase 1 Clarification**: Resolve Better Auth backend compatibility (if needed, adjust architecture)
4. **Phase 2 Kickoff**: Create tasks.md with specific task breakdown for engineers
5. **Implementation Begins**: Start with database schema, then backend, then frontend

---

## Appendices

### A. Technology Stack Summary

| Layer | Technology | Version | Purpose |
|-------|-----------|---------|---------|
| **Backend Runtime** | Python | 3.11 | Server runtime |
| **Backend Framework** | FastAPI | Latest | HTTP API server |
| **Database** | PostgreSQL | Neon-managed | User/session/metadata storage |
| **Password Hashing** | bcrypt | via passlib | Secure password storage |
| **JWT/Tokens** | python-jose | Latest | Token generation/validation |
| **Frontend Runtime** | Node.js | 18+ | Build/dev environment |
| **Frontend Framework** | React | 19 | UI framework |
| **Frontend Type System** | TypeScript | 5 | Type safety |
| **Auth Library** | Better Auth | ^1.4.5 | Frontend authentication client |
| **Form Handling** | react-hook-form | ^7.68 | Form state/validation |
| **HTTP Client** | axios | ^1.13 | API requests |

### B. Environment Variables Reference

**Backend (.env)**:
```
DATABASE_URL=postgresql://user:pass@host:5432/db
AUTH_SECRET=<random-32-char-secret>
PASSWORD_HASH_ROUNDS=12
JWT_ALGORITHM=HS256
JWT_EXPIRATION_HOURS=24
```

**Frontend (.env)**:
```
REACT_APP_API_URL=http://localhost:8001
REACT_APP_AUTH_ENDPOINT=/api/auth
```

### C. Deployment Checklist

- [ ] Backend auth requirements.txt updated
- [ ] Database migrations applied to production
- [ ] Environment variables set in production
- [ ] Frontend built with correct API_URL
- [ ] CORS headers include auth endpoints
- [ ] SSL/HTTPS enforced in production
- [ ] Monitoring/alerts configured for auth failures
- [ ] Backup strategy includes auth tables

---

**Plan Status**: ✅ Ready for Phase 1 Design Review

**Next Review Date**: After Phase 1 design work completes (estimated 2-3 days)

**Owner**: Project Architect / Tech Lead
