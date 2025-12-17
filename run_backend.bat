@echo off
REM Run RAG Chatbot Backend Server
REM Batch script to start the FastAPI backend with proper PYTHONPATH

REM Set PYTHONPATH to current directory
set PYTHONPATH=.

REM Display startup information
echo.
echo ============================================
echo   RAG Chatbot Backend Server
echo ============================================
echo PYTHONPATH: %PYTHONPATH%
echo API URL: http://localhost:8000
echo Health Check: curl http://localhost:8000/health
echo.
echo Type Ctrl+C to stop the server
echo ============================================
echo.

REM Run the FastAPI server
python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000 --reload

REM If script exits with error
if errorlevel 1 (
    echo.
    echo Error starting backend server.
    echo Check that requirements.txt dependencies are installed.
    echo Run: pip install -r requirements.txt
    pause
)
