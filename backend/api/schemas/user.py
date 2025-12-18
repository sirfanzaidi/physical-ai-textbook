"""
User profile request and response schemas.

These Pydantic models define the contracts for user profile endpoints.
"""

from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime


class ProfileUpdateRequest(BaseModel):
    """Request model for updating user profile."""

    name: Optional[str] = Field(
        default=None,
        min_length=1,
        max_length=255,
        description="User's full name"
    )

    # Background information
    programming_backgrounds: Optional[List[str]] = Field(
        default=None,
        description="List of programming languages/backgrounds"
    )
    frameworks_known: Optional[List[str]] = Field(
        default=None,
        description="List of frameworks known"
    )
    hardware_experience: Optional[List[str]] = Field(
        default=None,
        description="List of hardware platforms"
    )

    # Experience and interests
    robotics_interest: Optional[str] = Field(
        default=None,
        max_length=255,
        description="Interest in robotics"
    )
    experience_level: Optional[str] = Field(
        default=None,
        description="Experience level: beginner, intermediate, or advanced"
    )

    class Config:
        """Pydantic config for ProfileUpdateRequest."""
        json_schema_extra = {
            "example": {
                "name": "Jane Smith",
                "programming_backgrounds": ["Python", "JavaScript", "Rust"],
                "frameworks_known": ["React", "FastAPI", "Django"],
                "hardware_experience": ["Arduino", "Raspberry Pi", "Jetson"],
                "robotics_interest": "Bipedal locomotion",
                "experience_level": "advanced"
            }
        }


class ProfileResponse(BaseModel):
    """Response model for user profile."""

    id: str = Field(..., description="User ID")
    email: str = Field(..., description="User email")
    name: Optional[str] = Field(None, description="User's full name")

    # Background information
    programming_backgrounds: List[str] = Field(
        default_factory=list,
        description="Programming backgrounds"
    )
    frameworks_known: List[str] = Field(
        default_factory=list,
        description="Frameworks known"
    )
    hardware_experience: List[str] = Field(
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
        """Pydantic config for ProfileResponse."""
        from_attributes = True
        json_schema_extra = {
            "example": {
                "id": "550e8400-e29b-41d4-a716-446655440000",
                "email": "jane@example.com",
                "name": "Jane Smith",
                "programming_backgrounds": ["Python", "JavaScript"],
                "frameworks_known": ["React", "FastAPI"],
                "hardware_experience": ["Arduino", "Raspberry Pi"],
                "robotics_interest": "Bipedal locomotion",
                "experience_level": "advanced",
                "completed_onboarding": True,
                "created_at": "2025-12-18T10:00:00Z",
                "updated_at": "2025-12-18T10:05:00Z"
            }
        }
