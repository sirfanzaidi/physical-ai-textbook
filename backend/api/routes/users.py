"""
User profile routes for viewing and updating user information.

Endpoints:
- GET /api/users/profile - Get current user's profile
- PUT /api/users/profile - Update current user's profile
"""

from fastapi import APIRouter, HTTPException, status, Depends
from typing import Optional
import logging

from ..schemas.user import ProfileUpdateRequest, ProfileResponse
from ..schemas.auth import UserResponse
from ..services.user_service import UserService
from ..middleware.auth import get_current_user
from ..config import Settings

logger = logging.getLogger(__name__)

# Will be initialized in main app
user_service: Optional[UserService] = None
settings: Optional[Settings] = None

router = APIRouter(prefix="/api/users", tags=["users"])


def set_services(user_svc: UserService, config: Settings):
    """Initialize services (called from main app)."""
    global user_service, settings
    user_service = user_svc
    settings = config


@router.get(
    "/profile",
    response_model=ProfileResponse,
    responses={
        401: {"description": "Not authenticated"},
        404: {"description": "User not found"},
        500: {"description": "Server error"}
    }
)
async def get_profile(user_info: dict = Depends(get_current_user)) -> ProfileResponse:
    """
    Get current user's profile information.

    Returns the authenticated user's profile with all background information.

    Args:
        user_info: Current user info from auth middleware

    Returns:
        ProfileResponse with complete user profile

    Raises:
        HTTPException(401): If not authenticated
        HTTPException(404): If user not found
        HTTPException(500): If server error
    """
    try:
        user_id = user_info.get("user_id")

        # Get user from database
        user = user_service.get_user_by_id(user_id)

        if not user:
            logger.warning(f"Profile request for non-existent user: {user_id}")
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail={
                    "error": "User not found",
                    "code": "USER_NOT_FOUND"
                }
            )

        return ProfileResponse(**user.to_dict())

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get profile error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "An error occurred retrieving profile",
                "code": "SERVER_ERROR"
            }
        )


@router.put(
    "/profile",
    response_model=ProfileResponse,
    responses={
        400: {"description": "Validation error"},
        401: {"description": "Not authenticated"},
        404: {"description": "User not found"},
        500: {"description": "Server error"}
    }
)
async def update_profile(
    request: ProfileUpdateRequest,
    user_info: dict = Depends(get_current_user)
) -> ProfileResponse:
    """
    Update current user's profile information.

    Allows users to update any profile field. All fields are optional.
    Only provided fields will be updated.

    Args:
        request: Profile update request with fields to update
        user_info: Current user info from auth middleware

    Returns:
        ProfileResponse with updated profile

    Raises:
        HTTPException(400): If validation fails
        HTTPException(401): If not authenticated
        HTTPException(404): If user not found
        HTTPException(500): If server error
    """
    try:
        user_id = user_info.get("user_id")

        # Validate request
        experience_level = request.experience_level
        if experience_level:
            valid_levels = ["beginner", "intermediate", "advanced"]
            if experience_level not in valid_levels:
                logger.warning(f"Update profile validation failed: invalid experience level - {experience_level}")
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail={
                        "error": f"Experience level must be one of: {', '.join(valid_levels)}",
                        "code": "VALIDATION_ERROR"
                    }
                )

        # Build update data (only include non-None fields)
        update_data = {}
        if request.name is not None:
            update_data["name"] = request.name
        if request.programming_backgrounds is not None:
            update_data["programming_backgrounds"] = request.programming_backgrounds
        if request.frameworks_known is not None:
            update_data["frameworks_known"] = request.frameworks_known
        if request.hardware_experience is not None:
            update_data["hardware_experience"] = request.hardware_experience
        if request.robotics_interest is not None:
            update_data["robotics_interest"] = request.robotics_interest
        if request.experience_level is not None:
            update_data["experience_level"] = request.experience_level

        # Update user profile
        updated_user = user_service.update_user_profile(user_id, update_data)

        if not updated_user:
            logger.warning(f"Profile update failed for user: {user_id}")
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail={
                    "error": "User not found or update failed",
                    "code": "USER_NOT_FOUND"
                }
            )

        logger.info(f"User profile updated: {user_id}")

        return ProfileResponse(**updated_user.to_dict())

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Update profile error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "An error occurred updating profile",
                "code": "SERVER_ERROR"
            }
        )
