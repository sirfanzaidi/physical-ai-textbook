# CI/CD Setup Guide: GitHub Actions Workflows

## Overview

This guide documents the GitHub Actions workflows configured for the RAG Chatbot project, providing automated testing, validation, and quality checks on every push and pull request.

**Workflows Included**:
1. **E2E Tests** (`e2e-tests.yml`) — Frontend integration testing
2. **Backend Tests** (`backend-tests.yml`) — Python unit, integration, and requirement validation tests

---

## Workflows

### 1. E2E Tests Workflow (`e2e-tests.yml`)

**Purpose**: Automated end-to-end testing for the select-text feature and chat widget integration

**Triggers**:
- Push to `main`, `develop`, `001-rag-chatbot` branches
- Pull requests to `main`, `develop`, `001-rag-chatbot` branches
- Only when relevant files change (frontend, E2E tests, playwright config)

**Jobs**:

#### 1a. E2E Tests (Desktop Browsers)
Runs E2E tests on 3 desktop browsers in parallel:

```yaml
matrix:
  browser: [chromium, firefox, webkit]
```

**Steps**:
1. Checkout code
2. Setup Node.js (18.x)
3. Install npm dependencies
4. Install Playwright browsers
5. Start backend (FastAPI on localhost:8000)
6. Start frontend dev server (localhost:3000)
7. Run E2E tests for specific browser
8. Upload test reports and artifacts

**Test Coverage**:
- ✅ Text Selection Detection (4 tests)
- ✅ UI Validation Rules (3 tests)
- ✅ Chat Widget Integration (3 tests)
- ✅ Backend Validation HTTP 400 (3 tests)
- ✅ Zero-Leakage Validation (2 tests)
- ✅ Message History & Context (3 tests)
- ✅ Error Handling & Edge Cases (3 tests)

**Outputs**:
- `playwright-report-{browser}/` — Detailed HTML reports with videos/screenshots
- `test-results-{browser}/` — JUnit XML results
- GitHub PR comment with test summary

#### 1b. Mobile Browser Tests
Runs E2E tests on mobile browser emulations:

**Browsers**:
- Pixel 5 (Chrome mobile)
- iPhone 12 (Safari mobile)

**Tests**: Same suite as desktop, filtered by `@mobile` tag

**Outputs**:
- `playwright-report-mobile/` — Mobile test reports
- `test-results-mobile/` — Mobile test results

#### 1c. Test Summary Job
Aggregates results and posts to PR:

**Output**: Markdown comment on PR with:
- Overall pass/fail status
- Test coverage summary
- Links to detailed reports
- Artifacts information

---

### 2. Backend Tests Workflow (`backend-tests.yml`)

**Purpose**: Automated Python testing with coverage analysis and requirement validation

**Triggers**:
- Push to `main`, `develop`, `001-rag-chatbot` branches
- Pull requests to `main`, `develop`, `001-rag-chatbot` branches
- Only when backend files change

**Jobs**:

#### 2a. Backend Unit & Integration Tests
Runs comprehensive Python test suite

**Test Matrix**:
```yaml
python-version: ['3.11', '3.12']
```

**Services**:
- PostgreSQL 15 for database tests

**Steps**:
1. Checkout code
2. Setup Python
3. Install dependencies
4. Lint with flake8
5. Type check with mypy
6. Run unit tests with coverage
7. Run integration tests with coverage
8. Upload coverage to Codecov
9. Generate coverage badge
10. Upload test artifacts

**Test Coverage**:
- **Unit Tests**: Chunking, embedding, retrieval, generation, models
- **Integration Tests**: End-to-end pipelines, ingest→query flows
- **Coverage Target**: >80%

**Outputs**:
- `htmlcov/` — HTML coverage report
- `coverage.svg` — Coverage badge
- Codecov integration with coverage tracking

#### 2b. Select-Text Validation Tests (FR-007a)
Dedicated job for FR-007a validation

**Tests**:
- Select-text minimum 10 character enforcement
- HTTP 400 error responses with exact messages
- Validation error handling and recovery

**Command**:
```bash
pytest backend/tests/unit/test_select_text_filtering.py -v
```

**Outputs**:
- Test pass/fail status in workflow summary
- PR comment with FR-007a status

#### 2c. Chunking Validation Tests (FR-001a)
Dedicated job for FR-001a validation

**Tests**:
- Atomic unit preservation (code blocks, tables, equations)
- No mid-unit splitting
- Boundary preservation

**Command**:
```bash
pytest backend/tests/unit/test_chunker.py::test_atomic_unit_preservation -v
```

**Outputs**:
- Test pass/fail status in workflow summary
- PR comment with FR-001a status

#### 2d. Test Summary Job
Aggregates backend test results

**Output**: Markdown comment on PR with:
- Unit test results
- FR-007a validation status
- FR-001a validation status
- Requirements coverage summary
- Links to detailed reports

---

## Setup Instructions

### 1. Copy Workflow Files

Ensure the following files exist in your repository:

```
.github/workflows/
├── e2e-tests.yml
├── backend-tests.yml
```

These are already committed to the `001-rag-chatbot` branch.

### 2. Configure Repository Settings

In GitHub repository settings:

**Settings → Actions → General**:
- ✅ Allow all actions and reusable workflows
- ✅ Read and write permissions for workflows

**Settings → Environments** (optional):
- Create `production` environment for production deployments
- Set environment protection rules (require approvals, etc.)

### 3. Add Secrets (if needed)

In GitHub repository settings → **Secrets and variables → Actions**:

```bash
CODECOV_TOKEN          # For Codecov integration (optional)
SLACK_WEBHOOK_URL      # For Slack notifications (optional)
```

Current workflows don't require secrets (runs locally against localhost).

### 4. Configure Branch Protection

In GitHub repository settings → **Branches → Branch protection rules**:

For `main` and `develop` branches:

```yaml
Require status checks to pass before merging:
  - E2E Tests (chromium)
  - E2E Tests (firefox)
  - E2E Tests (webkit)
  - Backend Unit & Integration Tests (3.11)
  - Backend Unit & Integration Tests (3.12)
  - Select-Text Validation Tests (FR-007a)
  - Chunking Validation Tests (FR-001a)

Require code reviews before merging: 1 approval
Require branches to be up to date: Yes
```

This ensures all tests pass before code is merged.

---

## Running Tests Locally

### Run E2E Tests Locally

```bash
# Install dependencies
npm install --save-dev @playwright/test typescript ts-node

# Run all E2E tests
npm run test:e2e

# Run with UI inspector (recommended for development)
npm run test:e2e:ui

# Run specific browser
npx playwright test --project=chromium

# Run in debug mode
npm run test:e2e:debug

# View report
npm run test:e2e:report
```

### Run Backend Tests Locally

```bash
# Install dependencies
pip install -r requirements.txt
pip install pytest pytest-cov pytest-asyncio

# Run all tests
pytest backend/tests/ -v --cov=backend

# Run unit tests only
pytest backend/tests/unit/ -v --cov=backend

# Run integration tests only
pytest backend/tests/integration/ -v --cov=backend

# Run FR-007a tests
pytest backend/tests/unit/test_select_text_filtering.py -v

# Run FR-001a tests
pytest backend/tests/unit/test_chunker.py -v

# Generate coverage report
pytest --cov=backend --cov-report=html
open htmlcov/index.html
```

---

## Understanding Workflow Output

### GitHub Actions Dashboard

Access workflows at: `https://github.com/{owner}/{repo}/actions`

**Status indicators**:
- ✅ Green: All checks passed
- ❌ Red: At least one check failed
- ⏳ Yellow: Tests in progress
- ⊘ Grey: Skipped or not run

### PR Comments

Workflows post automatic comments on pull requests with:

```markdown
## E2E Test Results

✅ All tests passed

### Test Coverage
- Text Selection Detection
- UI Validation Rules (FR-007a)
- Chat Widget Integration
- Backend Validation (HTTP 400)
- Zero-Leakage Validation
- Message History & Context
- Cross-Browser Compatibility
- Error Handling & Edge Cases

### Test Status
- Unit Tests: ✅
- FR-007a (Select-Text): ✅
- FR-001a (Chunking): ✅

[View Detailed Report](https://github.com/...)
```

### Test Artifacts

After tests complete, artifacts are available for download:

**E2E Tests**:
- `playwright-report-{browser}` — HTML report with screenshots/videos
- `test-results-{browser}` — JUnit XML results

**Backend Tests**:
- `backend-test-results-py{version}` — Coverage reports
- HTML coverage reports with line-by-line analysis

Download from: **Actions → Run → Artifacts**

---

## Troubleshooting

### Tests Fail with "Backend failed to start"

**Issue**: Backend server didn't start in time

**Solution**:
1. Check if `requirements.txt` includes all dependencies
2. Increase wait timeout in workflow (currently 30 seconds)
3. Check backend logs in workflow output

```yaml
# In workflow, increase timeout
- name: Start backend server
  run: |
    python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000 &
    sleep 10  # Increased from 5 to 10
    for i in {1..60}; do  # Increased retries
      if curl -s http://localhost:8000/health > /dev/null; then
        exit 0
      fi
      sleep 1
    done
```

### Tests Fail with "Frontend failed to start"

**Issue**: Frontend dev server didn't start

**Solution**:
1. Verify `npm run dev` works locally
2. Check `package.json` has dev script
3. Increase wait timeout (currently 60 seconds)

### Flaky Tests (Intermittent Failures)

**Issue**: Tests pass locally but fail in CI

**Solution**:
1. Increase `timeout-minutes` in job config
2. Add explicit waits for page elements
3. Review Playwright reports for timing issues
4. Check for race conditions in tests

### Coverage Below Threshold

**Issue**: Code coverage <80%

**Solution**:
1. Add tests for uncovered code
2. Verify test file paths match coverage glob
3. Check `pytest.ini` or `pyproject.toml` coverage settings

```python
# Example: pyproject.toml
[tool.pytest.ini_options]
addopts = "--cov=backend --cov-report=html --cov-report=term"
testpaths = ["backend/tests"]
```

### Python Version Compatibility Issues

**Issue**: Tests fail with Python 3.12

**Solution**:
1. Check for deprecated Python APIs
2. Update dependencies to Python 3.12 compatible versions
3. Test locally with `python3.12 -m pytest`

---

## Performance Optimization

### Reduce Test Execution Time

**Parallel Execution**:
```yaml
# In E2E tests: Already uses matrix strategy (3 browsers in parallel)
# In Backend tests: Use pytest-xdist
pytest backend/tests/ -n auto  # Auto-detect CPU cores
```

**Cache Dependencies**:
```yaml
# Already configured in workflows
- uses: actions/setup-node@v4
  with:
    cache: 'npm'  # Caches node_modules

- uses: actions/setup-python@v4
  with:
    cache: 'pip'  # Caches pip packages
```

**Concurrency Control**:
```yaml
concurrency:
  group: ${{ github.workflow }}-${{ github.ref }}
  cancel-in-progress: true
```

This cancels previous runs when new commits are pushed.

---

## Monitoring & Alerts

### Slack Notifications (Optional)

Add Slack notification step:

```yaml
- name: Slack notification
  if: always()
  uses: slackapi/slack-github-action@v1
  with:
    payload: |
      {
        "text": "E2E Tests: ${{ job.status }}",
        "blocks": [
          {
            "type": "section",
            "text": {
              "type": "mrkdwn",
              "text": "*E2E Test Results*\nStatus: ${{ job.status }}"
            }
          }
        ]
      }
  env:
    SLACK_WEBHOOK_URL: ${{ secrets.SLACK_WEBHOOK_URL }}
```

### Codecov Integration

Current workflow integrates with Codecov:

```yaml
- name: Upload coverage reports
  uses: codecov/codecov-action@v3
```

View coverage trends at: `https://codecov.io/gh/{owner}/{repo}`

### GitHub Status Checks

All workflow jobs appear as status checks on PRs:

- ✅ Passed: Green checkmark, ready to merge
- ❌ Failed: Red X, must fix before merging
- ⏳ In Progress: Yellow dot, tests still running

---

## CI/CD Best Practices

### 1. Fail Fast

Current workflows fail fast on first error:
```yaml
fail-fast: false  # Actually allows all to run, then report all failures
```

Change to `true` to cancel remaining tests when one fails.

### 2. Artifact Retention

Workflows keep artifacts for 30 days:
```yaml
retention-days: 30
```

Adjust based on storage needs.

### 3. Timeout Management

Current timeouts:
- E2E Tests: 30 minutes per job
- Backend Tests: 20 minutes per job

Increase if tests take longer:
```yaml
timeout-minutes: 45
```

### 4. Environment Variables

Tests use environment variables:
```yaml
env:
  CI: true
  BASE_URL: http://localhost:3000
  API_URL: http://localhost:8000
  PYTHONPATH: ${{ github.workspace }}
```

Add more as needed without committing secrets.

---

## Extending Workflows

### Add Visual Regression Tests

```yaml
- name: Run visual regression tests
  run: npx playwright test --grep @visual
```

### Add Accessibility Tests

```yaml
- name: Run accessibility tests
  run: npm run test:a11y
```

### Add Load Testing

```yaml
- name: Run load tests
  run: python -m pytest backend/tests/load/ -v
```

### Add Security Scanning

```yaml
- name: Run security scan
  uses: github/super-linter@v4
```

---

## Debugging Failed Workflows

### View Workflow Logs

1. Go to Actions tab
2. Click failing workflow run
3. Click job name to expand logs
4. Search for "error" or "failed"

### Download Artifacts for Analysis

1. Click job in workflow run
2. Scroll to "Artifacts" section
3. Download relevant artifacts
4. Review locally for detailed debugging

### Re-run Failed Workflows

1. Click "Re-run all jobs" or "Re-run failed jobs"
2. Helpful for flaky tests
3. Check if issue is consistent

### Enable Debug Logging

In workflow, add:
```yaml
- name: Enable debug logging
  run: |
    echo "PLAYWRIGHT_DEBUG=pw:api" >> $GITHUB_ENV
    echo "DEBUG=pw:*" >> $GITHUB_ENV
```

---

## Next Steps

1. ✅ Workflows are configured and committed
2. ✅ Branch protection rules ready to configure
3. ✅ Local testing setup documented
4. ⏭️ Monitor first test runs on PRs
5. ⏭️ Tune timeouts based on actual execution times
6. ⏭️ Add optional integrations (Slack, Codecov badges)
7. ⏭️ Extend with additional test types (visual, load, security)

---

## Support

For questions or issues:
1. Check GitHub Actions documentation: https://docs.github.com/actions
2. Review workflow logs in Actions tab
3. Test locally with same commands as CI/CD
4. Check Playwright documentation: https://playwright.dev/docs/ci

---

## Summary

**CI/CD Workflows Configured**:
- ✅ E2E Tests: 3 desktop browsers + 2 mobile emulators
- ✅ Backend Tests: Python 3.11 & 3.12 with coverage
- ✅ FR-007a Validation: Select-text minimum 10 characters
- ✅ FR-001a Validation: Atomic unit preservation
- ✅ PR Comments: Automatic test result summaries
- ✅ Artifact Storage: Reports, videos, coverage for 30 days

**Ready for**:
- ✅ Automatic testing on every PR
- ✅ Branch protection enforcement
- ✅ Team visibility into test status
- ✅ Requirement compliance validation
- ✅ Production deployment gates
