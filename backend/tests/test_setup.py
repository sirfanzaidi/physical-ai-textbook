"""Test client initialization and configuration"""
import os

def test_environment_variables():
    """Test that environment variables can be loaded"""
    assert os.getenv("HOME") is not None

def test_imports():
    """Test that key libraries are importable"""
    try:
        import httpx
        import pydantic
        import logging
    except ImportError as e:
        raise ImportError(f"Missing library: {e}")

if __name__ == "__main__":
    test_environment_variables()
    test_imports()
    print("✓ Setup tests pass!")
