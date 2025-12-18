"""
Authentication service for handling password hashing, token generation, and verification.

This module provides secure password hashing using bcrypt and JWT token management.
"""

from passlib.context import CryptContext
from jose import JWTError, jwt
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
import os

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


class AuthService:
    """Service for authentication operations."""

    def __init__(
        self,
        secret_key: Optional[str] = None,
        algorithm: str = "HS256",
        access_token_expire_hours: int = 24
    ):
        """
        Initialize auth service.

        Args:
            secret_key: JWT secret key (default: from AUTH_SECRET env var)
            algorithm: JWT algorithm (default: HS256)
            access_token_expire_hours: Token expiration in hours (default: 24)
        """
        self.secret_key = secret_key or os.getenv("AUTH_SECRET", "your-secret-key-change-in-production")
        self.algorithm = algorithm
        self.access_token_expire_hours = access_token_expire_hours

        if self.secret_key == "your-secret-key-change-in-production":
            print("[WARNING] Using default AUTH_SECRET. Set AUTH_SECRET environment variable in production.")

    @staticmethod
    def hash_password(password: str) -> str:
        """
        Hash a password using bcrypt.

        Args:
            password: Plaintext password

        Returns:
            Bcrypt hashed password
        """
        return pwd_context.hash(password)

    @staticmethod
    def verify_password(password: str, password_hash: str) -> bool:
        """
        Verify a password against its hash.

        Args:
            password: Plaintext password to verify
            password_hash: Bcrypt hash to compare against

        Returns:
            True if password matches hash, False otherwise
        """
        return pwd_context.verify(password, password_hash)

    def generate_token(self, user_id: str, expires_hours: Optional[int] = None) -> tuple[str, datetime]:
        """
        Generate a JWT token for a user.

        Args:
            user_id: The user's ID to embed in token
            expires_hours: Token expiration in hours (default: instance setting)

        Returns:
            Tuple of (token_string, expiration_datetime)

        Raises:
            ValueError: If user_id is empty
        """
        if not user_id:
            raise ValueError("user_id cannot be empty")

        expires_hours = expires_hours or self.access_token_expire_hours
        now = datetime.utcnow()
        expires_at = now + timedelta(hours=expires_hours)

        payload = {
            "sub": user_id,  # Subject (user ID)
            "iat": now,      # Issued at
            "exp": expires_at # Expiration
        }

        token = jwt.encode(payload, self.secret_key, algorithm=self.algorithm)
        return token, expires_at

    def verify_token(self, token: str) -> Optional[Dict[str, Any]]:
        """
        Verify and decode a JWT token.

        Args:
            token: JWT token to verify

        Returns:
            Decoded token payload if valid, None otherwise

        Raises:
            JWTError: If token is malformed
        """
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm])
            user_id = payload.get("sub")
            if not user_id:
                return None
            return payload
        except JWTError:
            return None

    def refresh_token(self, token: str, refresh_window_minutes: int = 5) -> Optional[tuple[str, datetime]]:
        """
        Refresh an expired or expiring token.

        This allows users to stay logged in even if their token expired, as long as
        it's within the refresh window.

        Args:
            token: Expired JWT token
            refresh_window_minutes: How many minutes after expiration token can be refreshed

        Returns:
            Tuple of (new_token, new_expiration) if refreshable, None otherwise
        """
        try:
            # Decode with verification disabled to check expiration
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm], options={"verify_exp": False})

            exp_time = datetime.fromtimestamp(payload.get("exp", 0))
            now = datetime.utcnow()

            # Check if token expired within refresh window
            if (now - exp_time).total_seconds() > (refresh_window_minutes * 60):
                return None

            # Token is refreshable, generate new one
            user_id = payload.get("sub")
            if not user_id:
                return None

            return self.generate_token(user_id)

        except JWTError:
            return None


class AuthError(Exception):
    """Base exception for authentication errors."""
    pass


class InvalidCredentialsError(AuthError):
    """Raised when email/password credentials are invalid."""
    pass


class UserNotFoundError(AuthError):
    """Raised when user is not found in database."""
    pass


class UserAlreadyExistsError(AuthError):
    """Raised when user with email already exists."""
    pass


class InvalidTokenError(AuthError):
    """Raised when token is invalid or expired."""
    pass


# Global auth service instance
auth_service = AuthService()
