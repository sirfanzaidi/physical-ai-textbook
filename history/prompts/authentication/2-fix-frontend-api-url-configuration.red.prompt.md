---
id: 2
title: Fix frontend API URL configuration
stage: red
date: 2025-12-10
surface: agent
model: claude-haiku-4-5-20251001
feature: authentication
branch: main
user: user
command: Debug why signup/signin fail
labels: ["bug-fix", "api-configuration", "frontend"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - website/.env.local
  - website/src/pages/signin.tsx
  - website/src/pages/signup.tsx
  - website/src/context/AuthContext.tsx
tests:
  - Verified backend API works on port 8001
  - Confirmed signup endpoint returns 200
  - Confirmed signin endpoint returns 200 with valid credentials
  - Confirmed error handling for invalid credentials
---

## Prompt

but why user cannot signin successfully

## Response snapshot

Identified root cause: Frontend was making relative API calls (/api/auth/signup) which resolve to http://localhost:3000 (frontend port), but backend runs on http://localhost:8001. Fixed by:
1. Creating .env.local with REACT_APP_API_URL=http://localhost:8001
2. Updating all API calls in signin.tsx, signup.tsx, and AuthContext.tsx to use environment variable with fallback to /api for production

## Outcome

- ✅ Impact: Frontend will now correctly connect to backend on port 8001
- 🧪 Tests: Manual API testing confirmed all endpoints work correctly
- 📁 Files: 4 files modified (.env.local created, 3 files updated for env var usage)
- 🔁 Next prompts: User to restart frontend with `npm start` to apply environment variables
- 🧠 Reflection: Issue was API URL mismatch between frontend and backend ports. Solution follows standard Node.js conventions with REACT_APP_ prefix for environment variables.

## Evaluation notes (flywheel)

- Failure modes observed: Frontend hardcoded /api paths resolved to wrong port; non-JSON response triggered fallback error message
- Graders run and results (PASS/FAIL): Backend API tests PASS; Frontend needs restart to verify
- Prompt variant (if applicable): null
- Next experiment: Monitor frontend logs after restart to confirm API connections succeed

