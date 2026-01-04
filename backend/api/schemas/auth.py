"""
Authentication request and response schemas.

These Pydantic models define the contracts for signup, signin, and session endpoints.
"""

from pydantic import BaseModel, EmailStr, Field
from typing import Optional, List
from datetime import datetime


class SignUpRequest(BaseModel):
    """Request model for user signup/registration."""

    email: EmailStr = Field(..., description="User email address (must be unique)")
    password: str = Field(..., min_length=8, max_length=255, description="Password (minimum 8 characters)")
    name: str = Field(..., min_length=1, max_length=255, description="User's full name")

    # Background information (optional)
    programming_backgrounds: Optional[List[str]] = Field(
        default_factory=list,
        description="List of programming languages/backgrounds"
    )
    frameworks_known: Optional[List[str]] = Field(
        default_factory=list,
        description="List of frameworks known"
    )
    hardware_experience: Optional[List[str]] = Field(
        default_factory=list,
        description="List of hardware platforms"
    )

    # Experience and interests (optional)
    robotics_interest: Optional[str] = Field(
        default=None,
        max_length=255,
        description="Interest in robotics"
    )
    experience_level: str = Field(
        default="beginner",
        description="Experience level: beginner, intermediate, or advanced"
    )

    class Config:
        """Pydantic config for SignUpRequest."""
        json_schema_extra = {
            "example": {
                "email": "user@example.com",
                "password": "securePassword123",
                "name": "John Doe",
                "programming_backgrounds": ["Python", "JavaScript"],
                "frameworks_known": ["React", "FastAPI"],
                "hardware_experience": ["Arduino", "Raspberry Pi"],
                "robotics_interest": "Humanoid robots",
                "experience_level": "intermediate"
            }
        }


class SignInRequest(BaseModel):
    """Request model for user login."""

    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., description="User password")

    class Config:
        """Pydantic config for SignInRequest."""
        json_schema_extra = {
            "example": {
                "email": "user@example.com",
                "password": "securePassword123"
            }
        }


class SignOutRequest(BaseModel):
    """Request model for user logout (typically empty or contains token)."""

    class Config:
        """Pydantic config for SignOutRequest."""
        json_schema_extra = {
            "example": {}
        }


class UserResponse(BaseModel):
    """Response model for user data (safe to expose to client)."""

    id: str = Field(..., description="User ID (UUID)")
    email: str = Field(..., description="User email address")
    name: Optional[str] = Field(None, description="User's full name")

    # Profile information
    programming_backgrounds: Optional[List[str]] = Field(
        default_factory=list,
        description="Programming backgrounds"
    )
    frameworks_known: Optional[List[str]] = Field(
        default_factory=list,
        description="Frameworks known"
    )
    hardware_experience: Optional[List[str]] = Field(
        default_factory=list,
        description="Hardware experience"
    )
    robotics_interest: Optional[str] = Field(
        None,
        description="Robotics interest"
    )
    experience_level: str = Field(
        default="beginner",
        description="Experience level"
    )
    completed_onboarding: bool = Field(
        default=False,
        description="Whether user completed onboarding"
    )

    # Metadata
    created_at: datetime = Field(..., description="Account creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")

    class Config:
        """Pydantic config for UserResponse."""
        from_attributes = True  # Allow ORM model conversion
        json_schema_extra = {
            "example": {
                "id": "550e8400-e29b-41d4-a716-446655440000",
                "email": "user@example.com",
                "name": "John Doe",
                "programming_backgrounds": ["Python", "JavaScript"],
                "frameworks_known": ["React", "FastAPI"],
                "hardware_experience": ["Arduino"],
                "robotics_interest": "Humanoid robots",
                "experience_level": "intermediate",
                "completed_onboarding": True,
                "created_at": "2025-12-18T10:00:00Z",
                "updated_at": "2025-12-18T10:05:00Z"
            }
        }


class AuthResponse(BaseModel):
    """Response model for successful authentication (signup/signin)."""

    user: UserResponse = Field(..., description="User data")
    token: str = Field(..., description="JWT authentication token")
    expires_at: datetime = Field(..., description="Token expiration timestamp")

    class Config:
        """Pydantic config for AuthResponse."""
        json_schema_extra = {
            "example": {
                "user": {
                    "id": "550e8400-e29b-41d4-a716-446655440000",
                    "email": "user@example.com",
                    "name": "John Doe",
                    "programming_backgrounds": ["Python"],
                    "frameworks_known": ["FastAPI"],
                    "hardware_experience": ["Arduino"],
                    "robotics_interest": "Humanoid robots",
                    "experience_level": "beginner",
                    "completed_onboarding": True,
                    "created_at": "2025-12-18T10:00:00Z",
                    "updated_at": "2025-12-18T10:00:00Z"
                },
                "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
                "expires_at": "2025-12-19T10:00:00Z"
            }
        }


class SessionResponse(BaseModel):
    """Response model for getting current session."""

    user: UserResponse = Field(..., description="Current authenticated user")

    class Config:
        """Pydantic config for SessionResponse."""
        json_schema_extra = {
            "example": {
                "user": {
                    "id": "550e8400-e29b-41d4-a716-446655440000",
                    "email": "user@example.com",
                    "name": "John Doe",
                    "programming_backgrounds": ["Python"],
                    "frameworks_known": ["FastAPI"],
                    "hardware_experience": ["Arduino"],
                    "robotics_interest": "Humanoid robots",
                    "experience_level": "beginner",
                    "completed_onboarding": True,
                    "created_at": "2025-12-18T10:00:00Z",
                    "updated_at": "2025-12-18T10:00:00Z"
                }
            }
        }


class ErrorResponse(BaseModel):
    """Response model for error responses."""

    error: str = Field(..., description="Error message")
    code: str = Field(..., description="Error code (e.g., INVALID_CREDENTIALS)")
    details: Optional[str] = Field(None, description="Additional error details")

    class Config:
        """Pydantic config for ErrorResponse."""
        json_schema_extra = {
            "examples": {
                "invalid_credentials": {
                    "error": "Invalid email or password",
                    "code": "INVALID_CREDENTIALS"
                },
                "user_exists": {
                    "error": "An account with this email already exists",
                    "code": "USER_ALREADY_EXISTS"
                },
                "validation_error": {
                    "error": "Validation failed",
                    "code": "VALIDATION_ERROR",
                    "details": "Password must be at least 8 characters"
                },
                "unauthorized": {
                    "error": "Authentication required",
                    "code": "UNAUTHORIZED"
                }
            }
        }


class SuccessResponse(BaseModel):
    """Generic success response."""

    success: bool = Field(default=True, description="Operation status")
    message: Optional[str] = Field(None, description="Optional success message")

    class Config:
        """Pydantic config for SuccessResponse."""
        json_schema_extra = {
            "example": {
                "success": True,
                "message": "User logged out successfully"
            }
        }
