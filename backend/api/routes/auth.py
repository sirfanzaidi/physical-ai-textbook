"""
Authentication routes for signup, signin, and session management.

Endpoints:
- POST /api/auth/signup - Register new user
- POST /api/auth/signin - Authenticate user
- POST /api/auth/signout - Logout user
- GET /api/auth/session - Get current session
"""

from fastapi import APIRouter, HTTPException, status, Depends, Request
from typing import Optional
import logging
from datetime import datetime

from ..schemas.auth import (
    SignUpRequest, SignInRequest, AuthResponse, UserResponse,
    SessionResponse, SuccessResponse, ErrorResponse
)
from ..schemas.user import ProfileUpdateRequest, ProfileResponse
from ..services.auth_service import auth_service, InvalidCredentialsError, UserAlreadyExistsError
from ..services.user_service import UserService
from ..middleware.auth import get_current_user
from ..config import Settings
import json

logger = logging.getLogger(__name__)

# Will be initialized in main app
user_service: Optional[UserService] = None
settings: Optional[Settings] = None

router = APIRouter(prefix="/api/auth", tags=["authentication"])


def set_services(user_svc: UserService, config: Settings):
    """Initialize services (called from main app)."""
    global user_service, settings
    user_service = user_svc
    settings = config


@router.post(
    "/signup",
    response_model=AuthResponse,
    status_code=status.HTTP_201_CREATED,
    responses={
        400: {"model": ErrorResponse, "description": "Validation error"},
        409: {"model": ErrorResponse, "description": "User already exists"},
        500: {"model": ErrorResponse, "description": "Server error"}
    }
)
async def signup(request: SignUpRequest, req: Request) -> AuthResponse:
    """
    Register a new user account.

    Validates email and password, checks for duplicates, creates user with profile,
    generates JWT token, and returns authenticated user.

    Args:
        request: SignUp request with email, password, and background info
        req: HTTP request (for logging IP address)

    Returns:
        AuthResponse with user data and JWT token

    Raises:
        HTTPException(400): If validation fails
        HTTPException(409): If email already exists
        HTTPException(500): If server error
    """
    try:
        # Validate request
        if not request.email or not request.password or not request.name:
            logger.warning(f"Signup validation failed: missing required fields from {req.client.host}")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail={
                    "error": "Email, password, and name are required",
                    "code": "VALIDATION_ERROR"
                }
            )

        # Validate password strength
        if len(request.password) < 8:
            logger.warning(f"Signup validation failed: weak password from {req.client.host}")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail={
                    "error": "Password must be at least 8 characters",
                    "code": "VALIDATION_ERROR"
                }
            )

        # Validate experience level
        valid_levels = ["beginner", "intermediate", "advanced"]
        if request.experience_level not in valid_levels:
            request.experience_level = "beginner"

        # Check if user already exists
        if user_service.user_exists(request.email):
            logger.warning(f"Signup failed: user already exists - {request.email}")
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail={
                    "error": "An account with this email already exists",
                    "code": "USER_ALREADY_EXISTS"
                }
            )

        # Hash password
        password_hash = auth_service.hash_password(request.password)

        # Create user with profile
        user = user_service.create_user(
            email=request.email,
            password_hash=password_hash,
            name=request.name,
            programming_backgrounds=request.programming_backgrounds,
            frameworks_known=request.frameworks_known,
            hardware_experience=request.hardware_experience,
            robotics_interest=request.robotics_interest,
            experience_level=request.experience_level
        )

        # Generate token
        token, expires_at = auth_service.generate_token(user.id)

        # Log event
        logger.info(f"User signup successful: {user.email}")

        # Return response
        return AuthResponse(
            user=UserResponse(**user.to_dict()),
            token=token,
            expires_at=expires_at
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Signup error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "An error occurred during signup",
                "code": "SERVER_ERROR"
            }
        )


@router.post(
    "/signin",
    response_model=AuthResponse,
    responses={
        401: {"model": ErrorResponse, "description": "Invalid credentials"},
        500: {"model": ErrorResponse, "description": "Server error"}
    }
)
async def signin(request: SignInRequest, req: Request) -> AuthResponse:
    """
    Authenticate user and create session.

    Validates email and password, generates JWT token if valid.

    Args:
        request: SignIn request with email and password
        req: HTTP request (for logging IP address)

    Returns:
        AuthResponse with user data and JWT token

    Raises:
        HTTPException(401): If credentials are invalid
        HTTPException(500): If server error
    """
    try:
        # Validate request
        if not request.email or not request.password:
            logger.warning(f"Signin validation failed: missing fields from {req.client.host}")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={
                    "error": "Invalid email or password",
                    "code": "INVALID_CREDENTIALS"
                }
            )

        # Get user by email
        user = user_service.get_user_by_email(request.email)

        if not user:
            logger.warning(f"Signin failed: user not found - {request.email} from {req.client.host}")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={
                    "error": "Invalid email or password",
                    "code": "INVALID_CREDENTIALS"
                }
            )

        # Verify password
        if not auth_service.verify_password(request.password, user.password_hash):
            logger.warning(f"Signin failed: invalid password - {request.email} from {req.client.host}")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={
                    "error": "Invalid email or password",
                    "code": "INVALID_CREDENTIALS"
                }
            )

        # Generate token
        token, expires_at = auth_service.generate_token(user.id)

        # Log event
        logger.info(f"User signin successful: {user.email}")

        # Return response
        return AuthResponse(
            user=UserResponse(**user.to_dict()),
            token=token,
            expires_at=expires_at
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Signin error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "An error occurred during signin",
                "code": "SERVER_ERROR"
            }
        )


@router.post(
    "/signout",
    response_model=SuccessResponse,
    responses={
        500: {"model": ErrorResponse, "description": "Server error"}
    }
)
async def signout(user_info: dict = Depends(get_current_user)) -> SuccessResponse:
    """
    Logout user (invalidate session).

    Logs the signout event. Token is invalidated by client deletion.
    (In production, could add token blacklist for server-side invalidation.)

    Args:
        user_info: Current user info from auth middleware

    Returns:
        Success response

    Raises:
        HTTPException(401): If not authenticated
        HTTPException(500): If server error
    """
    try:
        user_id = user_info.get("user_id")
        logger.info(f"User signout: {user_id}")

        return SuccessResponse(
            success=True,
            message="Logged out successfully"
        )

    except Exception as e:
        logger.error(f"Signout error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "An error occurred during signout",
                "code": "SERVER_ERROR"
            }
        )


@router.get(
    "/session",
    response_model=SessionResponse,
    responses={
        401: {"model": ErrorResponse, "description": "Not authenticated"},
        500: {"model": ErrorResponse, "description": "Server error"}
    }
)
async def get_session(user_info: dict = Depends(get_current_user)) -> SessionResponse:
    """
    Get current authenticated user session.

    Returns the authenticated user's profile information.

    Args:
        user_info: Current user info from auth middleware

    Returns:
        SessionResponse with user data

    Raises:
        HTTPException(401): If not authenticated
        HTTPException(500): If server error
    """
    try:
        user_id = user_info.get("user_id")

        # Get fresh user data from database
        user = user_service.get_user_by_id(user_id)

        if not user:
            logger.warning(f"Session request for non-existent user: {user_id}")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={
                    "error": "User not found",
                    "code": "USER_NOT_FOUND"
                }
            )

        return SessionResponse(
            user=UserResponse(**user.to_dict())
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Session error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "An error occurred",
                "code": "SERVER_ERROR"
            }
        )
