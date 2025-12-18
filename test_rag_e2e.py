#!/usr/bin/env python3
"""
End-to-end test for RAG chatbot.
Simple test with ASCII-only output (no emoji).
"""

import sys
import os
import json
import time
import subprocess
from pathlib import Path

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent))

def test_health_check():
    """Test backend health endpoint."""
    print("\n" + "="*60)
    print("TEST 1: Health Check")
    print("="*60)

    try:
        import httpx
        with httpx.Client(timeout=10) as client:
            response = client.get('http://localhost:8000/api/health')
            if response.status_code == 200:
                data = response.json()
                print("PASS: Health check successful")
                print(f"Status: {data.get('status')}")
                return True
            else:
                print(f"FAIL: Status {response.status_code}")
                return False
    except Exception as e:
        print(f"FAIL: {str(e)}")
        return False


def test_chat_query():
    """Test chat query endpoint."""
    print("\n" + "="*60)
    print("TEST 2: Chat Query (Full Book Mode)")
    print("="*60)

    try:
        import httpx

        query_data = {
            "query": "What is physical AI?",
            "book_id": "physical_ai",
            "mode": "full"
        }

        print(f"Query: {query_data['query']}")

        with httpx.Client(timeout=30) as client:
            response = client.post(
                'http://localhost:8000/api/chat',
                json=query_data
            )

            if response.status_code == 200:
                data = response.json()
                print("PASS: Chat query successful")
                print(f"Response length: {len(data.get('response', ''))}")
                print(f"Confidence: {data.get('confidence', 'N/A')}")
                print(f"Latency: {data.get('latency_ms', 'N/A')}ms")
                if data.get('citations'):
                    print(f"Citations: {len(data.get('citations', []))}")
                return True
            else:
                print(f"FAIL: Status {response.status_code}")
                print(f"Response: {response.text[:200]}")
                return False
    except Exception as e:
        print(f"FAIL: {str(e)}")
        return False


def test_streaming_chat():
    """Test streaming chat endpoint."""
    print("\n" + "="*60)
    print("TEST 3: Streaming Chat Response")
    print("="*60)

    try:
        import httpx

        query_data = {
            "query": "What are humanoid robots?",
            "book_id": "physical_ai",
            "mode": "full"
        }

        print(f"Query: {query_data['query']}")
        print("Streaming response:")

        chunks_received = 0
        with httpx.Client(timeout=60) as client:
            with client.stream(
                'POST',
                'http://localhost:8000/api/chat-stream',
                json=query_data
            ) as response:
                if response.status_code == 200:
                    for line in response.iter_lines():
                        if line.strip():
                            try:
                                chunk = json.loads(line)
                                chunks_received += 1
                                if chunk.get('response'):
                                    print(f"  [{chunks_received}] Got response text")
                                if chunk.get('citations'):
                                    print(f"  [{chunks_received}] Got {len(chunk.get('citations', []))} citations")
                                if chunk.get('latency_ms'):
                                    print(f"  [{chunks_received}] Latency: {chunk.get('latency_ms')}ms")
                            except json.JSONDecodeError:
                                pass

                    if chunks_received > 0:
                        print(f"PASS: Received {chunks_received} chunks")
                        return True
                    else:
                        print("FAIL: No chunks received")
                        return False
                else:
                    print(f"FAIL: Status {response.status_code}")
                    return False

    except Exception as e:
        print(f"FAIL: {str(e)}")
        return False


def test_selected_text_mode():
    """Test selected text mode."""
    print("\n" + "="*60)
    print("TEST 4: Selected Text Mode")
    print("="*60)

    try:
        import httpx

        query_data = {
            "query": "What does this say about robots?",
            "book_id": "physical_ai",
            "mode": "selected",
            "selected_text": "Robotics is a field of engineering"
        }

        print(f"Selected text: {query_data['selected_text'][:50]}...")
        print(f"Query: {query_data['query']}")

        with httpx.Client(timeout=30) as client:
            response = client.post(
                'http://localhost:8000/api/chat',
                json=query_data
            )

            if response.status_code == 200:
                data = response.json()
                print("PASS: Selected text query successful")
                print(f"Response length: {len(data.get('response', ''))}")
                return True
            else:
                print(f"FAIL: Status {response.status_code}")
                print(f"Response: {response.text[:200]}")
                return False
    except Exception as e:
        print(f"FAIL: {str(e)}")
        return False


def main():
    """Run all tests."""
    print("\n" + "="*60)
    print("RAG CHATBOT END-TO-END TEST")
    print("="*60)

    # Check if backend is running
    print("\nChecking if backend is running...")
    try:
        import httpx
        with httpx.Client(timeout=5) as client:
            response = client.get('http://localhost:8000/api/health')
            if response.status_code == 200:
                print("Backend is running.")
            else:
                print("Backend not responding properly.")
                return 1
    except Exception as e:
        print(f"Backend not running: {str(e)}")
        print("\nStarting backend...")

        # Set environment variables
        env = os.environ.copy()
        env['OPENROUTER_API_KEY'] = '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'
        env['QDRANT_URL'] = 'https://50e62b6a-4834-4f71-a943-f2253f88b027.us-east4-0.gcp.cloud.qdrant.io'
        env['QDRANT_API_KEY'] = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0._Ti7-HNEzuJSowdF7c9z6RSudCNnN_pceBLk9G6uc-U'

        try:
            subprocess.Popen(
                [sys.executable, '-m', 'uvicorn', 'backend.api.app:app', '--host', 'localhost', '--port', '8000'],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            print("Backend started. Waiting 5 seconds for startup...")
            time.sleep(5)
        except Exception as e:
            print(f"Could not start backend: {str(e)}")
            return 1

    # Run tests
    results = []
    results.append(("Health Check", test_health_check()))
    results.append(("Chat Query", test_chat_query()))
    results.append(("Streaming Chat", test_streaming_chat()))
    results.append(("Selected Text Mode", test_selected_text_mode()))

    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)

    passed = 0
    total = len(results)

    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"{status:6} | {test_name}")
        if result:
            passed += 1

    print("="*60)
    print(f"Results: {passed}/{total} tests passed")

    if passed == total:
        print("\nSUCCESS: All tests passed!")
        return 0
    else:
        print(f"\nFAILURE: {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
