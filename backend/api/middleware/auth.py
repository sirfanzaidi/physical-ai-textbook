"""
Authentication middleware for JWT token verification.

This module provides middleware and dependency functions for protecting routes.
"""

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer
from starlette.authentication import AuthCredentials
from typing import Optional, NamedTuple
import logging

from ..services.auth_service import auth_service, InvalidTokenError

class HTTPAuthCredentials(NamedTuple):
    """Simple HTTP auth credentials holder."""
    scheme: str
    credentials: str

logger = logging.getLogger(__name__)

# HTTP Bearer authentication scheme
security = HTTPBearer(auto_error=False)


async def get_current_user(credentials: Optional[HTTPAuthCredentials] = Depends(security)) -> dict:
    """
    Dependency function to extract and verify JWT token from request.

    This function should be used in route handlers that require authentication.

    Args:
        credentials: HTTP Authorization header credentials

    Returns:
        Decoded JWT payload containing user information

    Raises:
        HTTPException(401): If token is missing, invalid, or expired
    """
    if not credentials:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required",
            headers={"WWW-Authenticate": "Bearer"},
        )

    token = credentials.credentials

    # Verify token
    payload = auth_service.verify_token(token)

    if not payload:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user_id = payload.get("sub")

    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token claims",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return {"user_id": user_id, "payload": payload}


async def get_current_user_id(user_info: dict = Depends(get_current_user)) -> str:
    """
    Extract user ID from current user info.

    Convenience function for routes that only need user ID.

    Args:
        user_info: User info from get_current_user dependency

    Returns:
        User ID string
    """
    return user_info["user_id"]


class AuthenticationError(Exception):
    """Raised when authentication fails."""
    pass


class TokenExpiredError(AuthenticationError):
    """Raised when token has expired."""
    pass


class InvalidTokenError(AuthenticationError):
    """Raised when token is invalid."""
    pass


def verify_token_manually(token: str) -> Optional[dict]:
    """
    Manually verify a token without using dependency injection.

    Useful for non-route functions that need to verify tokens.

    Args:
        token: JWT token to verify

    Returns:
        Decoded payload if valid, None otherwise
    """
    return auth_service.verify_token(token)


def extract_token_from_header(auth_header: Optional[str]) -> Optional[str]:
    """
    Extract bearer token from Authorization header.

    Args:
        auth_header: Authorization header value

    Returns:
        Token string if valid format, None otherwise

    Examples:
        >>> extract_token_from_header("Bearer eyJhbGc...")
        "eyJhbGc..."
        >>> extract_token_from_header("Basic invalid")
        None
    """
    if not auth_header:
        return None

    parts = auth_header.split()

    if len(parts) != 2 or parts[0].lower() != "bearer":
        return None

    return parts[1]
