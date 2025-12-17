# PYTHONPATH and Module Import Fix

## Problem

When running the backend server with:
```bash
python -m uvicorn backend.app.main:app
```

You may encounter:
```
ModuleNotFoundError: No module named 'ingestion'
```

Or similar import errors for `backend.ingestion`, `utils`, `database`, etc.

## Root Cause

The `PYTHONPATH` environment variable is not set, so Python cannot resolve relative imports within the backend package.

When uvicorn starts the app, it imports `backend.app.main`, which then tries to import `backend.ingestion` (or just `ingestion` if using relative imports). Without the proper path setup, Python cannot find these modules.

## Solutions

### Solution 1: Set PYTHONPATH Before Running (Recommended)

**Windows (PowerShell)**:
```powershell
$env:PYTHONPATH = "D:\physical-ai-textbook"
python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000
```

**Windows (Command Prompt)**:
```cmd
set PYTHONPATH=D:\physical-ai-textbook
python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000
```

**macOS/Linux (Bash/Zsh)**:
```bash
export PYTHONPATH="/path/to/physical-ai-textbook"
python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000
```

### Solution 2: Run from Project Root with PYTHONPATH

**Single Command (PowerShell)**:
```powershell
$env:PYTHONPATH = "."; python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000
```

**Single Command (Bash)**:
```bash
PYTHONPATH=. python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000
```

### Solution 3: Create .env File with PYTHONPATH

Create or update `.env` file in project root:

```bash
# .env
PYTHONPATH=D:\physical-ai-textbook
```

Then run:
```powershell
python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000
```

The `python-dotenv` package (already in requirements) will load this automatically if configured in the app.

### Solution 4: Run Python Directly from Project Root

```bash
cd D:\physical-ai-textbook
python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000
```

If `PYTHONPATH` is already set or the current directory (`.`) is implicitly added to `sys.path`.

## Verify PYTHONPATH is Correct

Check if PYTHONPATH is set:

**PowerShell**:
```powershell
$env:PYTHONPATH
```

**Bash**:
```bash
echo $PYTHONPATH
```

Or check within Python:
```python
import sys
print(sys.path)
```

Should include the project root directory.

## Fix for CI/CD (GitHub Actions)

The GitHub Actions workflows already handle this correctly:

```yaml
env:
  PYTHONPATH: ${{ github.workspace }}
```

This sets `PYTHONPATH` to the repository root before running tests.

---

## Alternative: Fix Import Statements

If you want to avoid relying on `PYTHONPATH`, you can fix the import statements in the code.

### Current (Problematic):
```python
from utils.errors import EmbeddingError
from ingestion.chunker import ChunkerConfig
```

### Fixed (Relative Imports):
```python
from backend.utils.errors import EmbeddingError
from backend.ingestion.chunker import ChunkerConfig
```

Or (using relative imports):
```python
from ..utils.errors import EmbeddingError
from ..ingestion.chunker import ChunkerConfig
```

**Note**: This requires ensuring that imports throughout the codebase are consistent.

---

## Recommended: Use .env + python-dotenv

Create `.env` file:
```
PYTHONPATH=.
DEBUG=true
LOG_LEVEL=INFO
```

Update `backend/app/config.py` to load it:
```python
from pydantic_settings import BaseSettings
from dotenv import load_dotenv

load_dotenv()  # Loads .env file

class Settings(BaseSettings):
    # ... settings definition
```

Then run normally:
```bash
python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000
```

---

## Quick Fix Script

Save as `run_backend.sh` (or `.ps1` for PowerShell):

**Bash** (`run_backend.sh`):
```bash
#!/bin/bash
export PYTHONPATH="$PWD"
python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000
```

**PowerShell** (`run_backend.ps1`):
```powershell
$env:PYTHONPATH = "$PWD"
python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000
```

Then run:
```bash
./run_backend.sh        # Bash
.\run_backend.ps1       # PowerShell
```

---

## Testing

After setting `PYTHONPATH`, verify the backend starts:

```bash
curl http://localhost:8000/health
```

Should return:
```json
{"status": "healthy", "environment": "development"}
```

If you see `ModuleNotFoundError`, check:
1. ✅ PYTHONPATH is set correctly
2. ✅ You're running from the correct directory
3. ✅ All required dependencies are installed (`pip install -r requirements.txt`)
4. ✅ Python can find the modules (`python -c "import backend.ingestion"`)

