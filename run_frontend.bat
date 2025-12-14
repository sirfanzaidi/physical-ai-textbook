@echo off
REM Run RAG Chatbot Frontend Dev Server
REM Batch script to start the Docusaurus frontend dev server

echo.
echo ============================================
echo   RAG Chatbot Frontend Dev Server
echo ============================================
echo Frontend URL: http://localhost:3000
echo Type Ctrl+C to stop the server
echo ============================================
echo.

REM Change to website directory
cd website

REM Check if node_modules exist
if not exist node_modules (
    echo Installing dependencies...
    call npm install
    if errorlevel 1 (
        echo Error installing dependencies.
        pause
        exit /b 1
    )
)

REM Run the dev server
echo Starting frontend dev server...
call npm run start

REM If script exits with error
if errorlevel 1 (
    echo.
    echo Error starting frontend server.
    echo Check that npm dependencies are installed.
    echo Run: npm install
    pause
)
