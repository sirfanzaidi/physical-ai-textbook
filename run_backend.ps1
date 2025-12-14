# Run RAG Chatbot Backend Server
# PowerShell script to start the FastAPI backend with proper PYTHONPATH

# Set PYTHONPATH to current directory
$env:PYTHONPATH = "."

# Display startup information
Write-Host "Starting RAG Chatbot Backend Server..." -ForegroundColor Green
Write-Host "- PYTHONPATH: $env:PYTHONPATH" -ForegroundColor Cyan
Write-Host "- API URL: http://localhost:8000" -ForegroundColor Cyan
Write-Host "- Health Check: curl http://localhost:8000/health" -ForegroundColor Cyan
Write-Host ""

# Run the FastAPI server
python -m uvicorn backend.app.main:app --host 0.0.0.0 --port 8000 --reload

# If script exits, show error
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error starting backend server. Check requirements.txt dependencies." -ForegroundColor Red
    Write-Host "Run: pip install -r requirements.txt" -ForegroundColor Yellow
}
