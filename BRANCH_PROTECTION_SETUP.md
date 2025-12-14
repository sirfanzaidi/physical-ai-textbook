# GitHub Branch Protection Rules Setup Guide

## Overview

This guide walks you through configuring GitHub branch protection rules for the RAG Chatbot project. Branch protection ensures all code merged to `main` and `develop` branches meets quality standards through required status checks, code reviews, and dismissal settings.

---

## Prerequisites

- ✅ Repository owner or admin access on GitHub
- ✅ CI/CD workflows committed (`.github/workflows/e2e-tests.yml`, `backend-tests.yml`)
- ✅ Workflows have run at least once to register as status checks in GitHub

---

## Step 1: Navigate to Branch Protection Settings

1. Go to your GitHub repository: `https://github.com/YOUR_ORG/physical-ai-textbook`
2. Click **Settings** (gear icon, top right)
3. In left sidebar, click **Branches**
4. Click **Add rule** button

---

## Step 2: Create Branch Protection Rule for `main`

### 2.1 Basic Configuration

**Branch name pattern**: `main`

### 2.2 Require Status Checks to Pass Before Merging

✅ **Enable**: "Require status checks to pass before merging"

**Search and select the following status checks**:

```
E2E Tests (chromium)
E2E Tests (firefox)
E2E Tests (webkit)
E2E Tests (Mobile)
Backend Unit & Integration Tests (3.11)
Backend Unit & Integration Tests (3.12)
Select-Text Validation Tests (FR-007a)
Chunking Validation Tests (FR-001a)
E2E Test Summary
Backend Test Summary
```

**Additional Settings**:
- ✅ **Require branches to be up to date before merging**: Yes
  - This ensures tests run against latest main before merge

### 2.3 Require Code Reviews Before Merging

✅ **Enable**: "Require a pull request before merging"

- **Required number of approvals before merging**: `1`
- ✅ **Require review from code owners**: Yes (if CODEOWNERS file exists)
- ✅ **Dismiss stale pull request approvals when new commits are pushed**: Yes
- ✅ **Require approval of the most recent reviewable push**: Yes

### 2.4 Additional Protections

✅ **Require status checks to pass before merging**:
- ✅ **Require branches to be up to date before merging**: Yes

✅ **Restrict who can push to matching branches**:
- Leave unchecked (allows all collaborators with push access)

✅ **Allow force pushes**:
- ⭕ **Allow force pushes**: Restrict to admins OR Disable entirely (recommended)
- Recommendation: **Disable** (most restrictive)

✅ **Allow deletions**:
- ⭕ **Allow deletions**: Disable (prevents accidental branch deletion)
- Recommendation: **Disable**

✅ **Include administrators**:
- ✅ **Enforce all the above rules for administrators too**: Yes
  - This ensures consistency; admins must follow same rules

---

## Step 3: Create Branch Protection Rule for `develop`

**Repeat Step 2** with the following modifications:

**Branch name pattern**: `develop`

**Required status checks** (same as main):
```
E2E Tests (chromium)
E2E Tests (firefox)
E2E Tests (webkit)
E2E Tests (Mobile)
Backend Unit & Integration Tests (3.11)
Backend Unit & Integration Tests (3.12)
Select-Text Validation Tests (FR-007a)
Chunking Validation Tests (FR-001a)
E2E Test Summary
Backend Test Summary
```

**Code review settings**:
- **Required number of approvals**: `1` (can be same as main or higher)
- Same dismiss/review settings as main

**Additional protections**: Same as main

---

## Step 4: Create Branch Protection Rule for `001-rag-chatbot` (Optional)

For feature branch protection during active development:

**Branch name pattern**: `001-rag-chatbot`

**Required status checks** (same as main):
```
E2E Tests (chromium)
E2E Tests (firefox)
E2E Tests (webkit)
E2E Tests (Mobile)
Backend Unit & Integration Tests (3.11)
Backend Unit & Integration Tests (3.12)
Select-Text Validation Tests (FR-007a)
Chunking Validation Tests (FR-001a)
E2E Test Summary
Backend Test Summary
```

**Code review settings**:
- **Required number of approvals**: `0` (optional for feature branches)
- Can be more lenient than main/develop

**Additional protections**: Same as main

---

## Expected Status Checks in GitHub

Once workflows run, you should see these checks available in branch protection settings:

### E2E Tests Workflow
- ✅ `E2E Tests (chromium)`
- ✅ `E2E Tests (firefox)`
- ✅ `E2E Tests (webkit)`
- ✅ `E2E Tests (Mobile)`
- ✅ `E2E Test Summary`

### Backend Tests Workflow
- ✅ `Backend Unit & Integration Tests (3.11)`
- ✅ `Backend Unit & Integration Tests (3.12)`
- ✅ `Select-Text Validation Tests (FR-007a)`
- ✅ `Chunking Validation Tests (FR-001a)`
- ✅ `Backend Test Summary`

---

## Configuration Summary Table

| Setting | `main` | `develop` | `001-rag-chatbot` |
|---------|--------|-----------|-------------------|
| **Status Checks Required** | ✅ All 10 | ✅ All 10 | ✅ All 10 |
| **Up-to-date Required** | ✅ Yes | ✅ Yes | ✅ Yes |
| **Code Reviews Required** | ✅ 1 | ✅ 1 | ⭕ Optional (0) |
| **Dismiss Stale Reviews** | ✅ Yes | ✅ Yes | ✅ Yes |
| **Force Push Allowed** | ❌ No | ❌ No | ❌ No |
| **Deletions Allowed** | ❌ No | ❌ No | ❌ No |
| **Enforce for Admins** | ✅ Yes | ✅ Yes | ✅ Yes |

---

## Verification Checklist

After configuring branch protection rules, verify:

- [ ] Navigate to **Settings → Branches**
- [ ] Confirm rules exist for `main`, `develop`, `001-rag-chatbot`
- [ ] Verify all 10 status checks are required
- [ ] Verify code review requirement (1 approval)
- [ ] Create a test PR to `develop` and confirm status checks are enforced
- [ ] Attempt to merge without approvals → should be blocked
- [ ] Attempt to merge without passing tests → should be blocked
- [ ] Approve and merge a test PR → should succeed with all checks passing

---

## Troubleshooting

### Issue: Status checks not appearing in branch protection settings

**Cause**: Workflows haven't run yet on this branch

**Solution**:
1. Create a test PR to the branch
2. Workflows will trigger and run
3. Once complete, status checks will appear in branch protection settings
4. Return to Settings → Branches and add the checks

### Issue: "Require branches to be up to date" prevents merging

**Cause**: New commits pushed to main/develop after PR was created

**Solution**:
1. Click "Update branch" button on PR
2. Re-run workflows to ensure latest code passes tests
3. Then merge will be allowed

### Issue: Cannot find certain status checks

**Cause**: Workflow job names don't match exactly

**Check**: Verify job names in workflow files match:
- `.github/workflows/e2e-tests.yml`:
  - `e2e-tests` (job name, will appear as `E2E Tests (chromium)`, etc. based on matrix)
  - `e2e-tests-mobile` (job name, will appear as `E2E Tests (Mobile)`)
  - `test-summary` (job name, will appear as `E2E Test Summary`)

- `.github/workflows/backend-tests.yml`:
  - `backend-tests` (job name, will appear as `Backend Unit & Integration Tests (3.11)`, etc.)
  - `select-text-validation` (job name, will appear as `Select-Text Validation Tests (FR-007a)`)
  - `chunking-validation` (job name, will appear as `Chunking Validation Tests (FR-001a)`)
  - `test-summary` (job name, will appear as `Backend Test Summary`)

---

## What Happens After Protection is Configured

### Pull Request Workflow

1. **Create PR** to `main` or `develop`
2. **Workflows trigger automatically**:
   - E2E tests run on 5 browsers
   - Backend tests run on Python 3.11, 3.12
   - FR-007a and FR-001a validation runs
3. **Status checks appear on PR**:
   - ✅ All pass → PR is mergeable
   - ❌ Any fail → PR is blocked until fixed
4. **Code review required**:
   - At least 1 approval needed
   - Dismiss stale reviews if new commits added
5. **Branch must be up to date**:
   - If main has new commits, "Update branch" required
   - Tests re-run against latest main
6. **Merge allowed only when**:
   - ✅ All status checks pass
   - ✅ At least 1 approval obtained
   - ✅ Branch is up to date with main

### Example PR Comment from Workflows

Once workflows run, you'll see automatic PR comments like:

```markdown
## E2E Test Results

✅ All E2E tests passed

### Test Coverage
- Text Selection Detection
- UI Validation Rules (FR-007a)
- Chat Widget Integration
- Backend Validation (HTTP 400)
- Zero-Leakage Validation
- Message History & Context
- Cross-Browser Compatibility
- Error Handling & Edge Cases

[View Detailed Report](https://github.com/...)
```

---

## Best Practices

1. **Start with `develop` first**: Test branch protection on `develop` before applying to `main`
2. **Monitor workflow performance**: Adjust timeouts if workflows consistently fail
3. **Keep status checks lean**: Only require essential checks; avoid redundant checks
4. **Document exceptions**: If you need to bypass protection, document why (emergency fixes)
5. **Review quarterly**: Periodically review branch protection rules to ensure they're still appropriate
6. **Use CODEOWNERS**: Create `.github/CODEOWNERS` file to require specific team reviews for certain paths

---

## Creating a CODEOWNERS File (Optional)

Create `.github/CODEOWNERS` to require specific team/user reviews:

```
# Backend
/backend/                    @backend-team
/backend/app/api/           @api-team
/backend/ingestion/         @ingestion-team

# Frontend
/frontend/                   @frontend-team
/website/                    @docs-team

# Tests
/backend/tests/e2e/         @qa-team
/backend/tests/unit/        @backend-team

# Workflows
.github/workflows/          @devops-team
```

Then in branch protection settings, enable **"Require review from code owners"**.

---

## Enforcement Timeline

| Phase | Duration | Action |
|-------|----------|--------|
| **Phase 1: Soft Enforcement** | 1 week | Configure rules, monitor, don't enforce for admins |
| **Phase 2: Full Enforcement** | Ongoing | Enable for all (including admins), monitor metrics |
| **Phase 3: Escalation** | As needed | Add additional checks if issues arise (coverage, security scans) |

---

## Additional Resources

- [GitHub: Protecting important branches](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches/about-protected-branches)
- [GitHub: Requiring status checks before merging](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches/requiring-status-checks-before-merging)
- [GitHub: Requiring code reviews](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches/requiring-code-reviews-for-pull-requests)

---

## Summary

After completing this guide, you'll have:

✅ **`main` branch**: Fully protected with all status checks, code reviews, and admin enforcement
✅ **`develop` branch**: Fully protected with same standards for quality assurance
✅ **`001-rag-chatbot` branch**: Optional protection for active feature development
✅ **Automated quality gates**: Every PR validated against E2E tests, backend tests, FR-007a, FR-001a
✅ **Code review enforcement**: At least 1 approval required before merge
✅ **Up-to-date requirement**: Branches must be current with latest main before merge

All changes are now protected by comprehensive automated testing and manual code review requirements.
