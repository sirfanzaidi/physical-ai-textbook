"""
User service for CRUD operations and profile management.

This module handles user account creation, retrieval, and profile updates.
"""

from typing import Optional, Dict, Any, List
import json
from datetime import datetime
import logging

logger = logging.getLogger(__name__)


class User:
    """User model representing a user record."""

    def __init__(
        self,
        id: str,
        email: str,
        password_hash: str,
        name: Optional[str] = None,
        programming_backgrounds: Optional[List[str]] = None,
        frameworks_known: Optional[List[str]] = None,
        hardware_experience: Optional[List[str]] = None,
        robotics_interest: Optional[str] = None,
        experience_level: str = "beginner",
        completed_onboarding: bool = False,
        created_at: Optional[datetime] = None,
        updated_at: Optional[datetime] = None
    ):
        """Initialize a User object."""
        self.id = id
        self.email = email
        self.password_hash = password_hash
        self.name = name
        self.programming_backgrounds = programming_backgrounds or []
        self.frameworks_known = frameworks_known or []
        self.hardware_experience = hardware_experience or []
        self.robotics_interest = robotics_interest
        self.experience_level = experience_level
        self.completed_onboarding = completed_onboarding
        self.created_at = created_at or datetime.utcnow()
        self.updated_at = updated_at or datetime.utcnow()

    def to_dict(self, include_password: bool = False) -> Dict[str, Any]:
        """Convert user to dictionary (safe for JSON response)."""
        data = {
            "id": self.id,
            "email": self.email,
            "name": self.name,
            "programming_backgrounds": self.programming_backgrounds,
            "frameworks_known": self.frameworks_known,
            "hardware_experience": self.hardware_experience,
            "robotics_interest": self.robotics_interest,
            "experience_level": self.experience_level,
            "completed_onboarding": self.completed_onboarding,
            "created_at": self.created_at,
            "updated_at": self.updated_at
        }
        if include_password:
            data["password_hash"] = self.password_hash
        return data


class UserService:
    """Service for user management operations."""

    def __init__(self, db_pool):
        """
        Initialize user service.

        Args:
            db_pool: Database connection pool (psycopg2.pool.SimpleConnectionPool)
        """
        self.db_pool = db_pool

    def create_user(
        self,
        email: str,
        password_hash: str,
        name: str,
        programming_backgrounds: Optional[List[str]] = None,
        frameworks_known: Optional[List[str]] = None,
        hardware_experience: Optional[List[str]] = None,
        robotics_interest: Optional[str] = None,
        experience_level: str = "beginner"
    ) -> User:
        """
        Create a new user account.

        Args:
            email: User email (must be unique)
            password_hash: Hashed password (from auth_service)
            name: User's full name
            programming_backgrounds: List of programming languages
            frameworks_known: List of frameworks known
            hardware_experience: List of hardware platforms
            robotics_interest: Robotics interest description
            experience_level: Experience level (beginner/intermediate/advanced)

        Returns:
            Created User object

        Raises:
            ValueError: If email or password_hash is empty
            Exception: If database error occurs
        """
        if not email or not password_hash:
            raise ValueError("email and password_hash are required")

        conn = self.db_pool.getconn()
        try:
            # Insert user record
            user_sql = """
                INSERT INTO users (email, password_hash, name, created_at, updated_at)
                VALUES (%s, %s, %s, %s, %s)
                RETURNING id, email, password_hash, name, created_at, updated_at;
            """
            now = datetime.utcnow()

            with conn.cursor() as cur:
                cur.execute(user_sql, (email, password_hash, name, now, now))
                user_row = cur.fetchone()

            if not user_row:
                raise Exception("Failed to create user")

            user_id, _, _, _, _, _ = user_row

            # Insert user profile record
            profile_sql = """
                INSERT INTO user_profiles (
                    user_id, programming_backgrounds, frameworks_known,
                    hardware_experience, robotics_interest, experience_level,
                    completed_onboarding, created_at, updated_at
                )
                VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s);
            """

            programming_backgrounds = programming_backgrounds or []
            frameworks_known = frameworks_known or []
            hardware_experience = hardware_experience or []

            with conn.cursor() as cur:
                cur.execute(
                    profile_sql,
                    (
                        user_id,
                        json.dumps(programming_backgrounds),
                        json.dumps(frameworks_known),
                        json.dumps(hardware_experience),
                        robotics_interest,
                        experience_level,
                        True,  # completed_onboarding
                        now,
                        now
                    )
                )

            conn.commit()
            logger.info(f"User created: {email}")

            # Return created user
            return User(
                id=str(user_id),
                email=email,
                password_hash=password_hash,
                name=name,
                programming_backgrounds=programming_backgrounds,
                frameworks_known=frameworks_known,
                hardware_experience=hardware_experience,
                robotics_interest=robotics_interest,
                experience_level=experience_level,
                completed_onboarding=True,
                created_at=now,
                updated_at=now
            )

        except Exception as e:
            conn.rollback()
            logger.error(f"Error creating user: {str(e)}")
            raise
        finally:
            self.db_pool.putconn(conn)

    def get_user_by_email(self, email: str) -> Optional[User]:
        """
        Retrieve user by email address.

        Args:
            email: User email to search for

        Returns:
            User object if found, None otherwise
        """
        conn = self.db_pool.getconn()
        try:
            sql = """
                SELECT u.id, u.email, u.password_hash, u.name, u.created_at, u.updated_at,
                       up.programming_backgrounds, up.frameworks_known, up.hardware_experience,
                       up.robotics_interest, up.experience_level, up.completed_onboarding
                FROM users u
                LEFT JOIN user_profiles up ON u.id = up.user_id
                WHERE u.email = %s;
            """

            with conn.cursor() as cur:
                cur.execute(sql, (email,))
                row = cur.fetchone()

            if not row:
                return None

            return self._row_to_user(row)

        except Exception as e:
            logger.error(f"Error retrieving user by email: {str(e)}")
            return None
        finally:
            self.db_pool.putconn(conn)

    def get_user_by_id(self, user_id: str) -> Optional[User]:
        """
        Retrieve user by ID.

        Args:
            user_id: User ID to search for

        Returns:
            User object if found, None otherwise
        """
        conn = self.db_pool.getconn()
        try:
            sql = """
                SELECT u.id, u.email, u.password_hash, u.name, u.created_at, u.updated_at,
                       up.programming_backgrounds, up.frameworks_known, up.hardware_experience,
                       up.robotics_interest, up.experience_level, up.completed_onboarding
                FROM users u
                LEFT JOIN user_profiles up ON u.id = up.user_id
                WHERE u.id = %s;
            """

            with conn.cursor() as cur:
                cur.execute(sql, (user_id,))
                row = cur.fetchone()

            if not row:
                return None

            return self._row_to_user(row)

        except Exception as e:
            logger.error(f"Error retrieving user by ID: {str(e)}")
            return None
        finally:
            self.db_pool.putconn(conn)

    def update_user_profile(self, user_id: str, profile_data: Dict[str, Any]) -> Optional[User]:
        """
        Update user profile information.

        Args:
            user_id: User ID to update
            profile_data: Dictionary of profile fields to update

        Returns:
            Updated User object, or None if update fails
        """
        conn = self.db_pool.getconn()
        try:
            # Build dynamic UPDATE query
            updates = []
            values = []

            # Handle user table fields
            if "name" in profile_data:
                updates.append("name = %s")
                values.append(profile_data["name"])

            # Update timestamp
            updates.append("updated_at = %s")
            values.append(datetime.utcnow())

            if updates:
                user_sql = f"UPDATE users SET {', '.join(updates)} WHERE id = %s;"
                values.append(user_id)

                with conn.cursor() as cur:
                    cur.execute(user_sql, values)

            # Handle profile table fields
            profile_updates = []
            profile_values = []

            profile_fields = [
                "programming_backgrounds",
                "frameworks_known",
                "hardware_experience",
                "robotics_interest",
                "experience_level"
            ]

            for field in profile_fields:
                if field in profile_data:
                    profile_updates.append(f"{field} = %s")
                    value = profile_data[field]
                    # Convert lists to JSON
                    if isinstance(value, list):
                        profile_values.append(json.dumps(value))
                    else:
                        profile_values.append(value)

            # Always update timestamp
            profile_updates.append("updated_at = %s")
            profile_values.append(datetime.utcnow())

            if profile_updates:
                profile_sql = f"UPDATE user_profiles SET {', '.join(profile_updates)} WHERE user_id = %s;"
                profile_values.append(user_id)

                with conn.cursor() as cur:
                    cur.execute(profile_sql, profile_values)

            conn.commit()
            logger.info(f"User profile updated: {user_id}")

            # Return updated user
            return self.get_user_by_id(user_id)

        except Exception as e:
            conn.rollback()
            logger.error(f"Error updating user profile: {str(e)}")
            return None
        finally:
            self.db_pool.putconn(conn)

    def user_exists(self, email: str) -> bool:
        """
        Check if user with email already exists.

        Args:
            email: Email to check

        Returns:
            True if user exists, False otherwise
        """
        conn = self.db_pool.getconn()
        try:
            sql = "SELECT 1 FROM users WHERE email = %s LIMIT 1;"

            with conn.cursor() as cur:
                cur.execute(sql, (email,))
                result = cur.fetchone()

            return result is not None

        except Exception as e:
            logger.error(f"Error checking if user exists: {str(e)}")
            return False
        finally:
            self.db_pool.putconn(conn)

    @staticmethod
    def _row_to_user(row: tuple) -> User:
        """Convert database row to User object."""
        (
            user_id, email, password_hash, name, created_at, updated_at,
            programming_backgrounds_json, frameworks_known_json, hardware_experience_json,
            robotics_interest, experience_level, completed_onboarding
        ) = row

        # Handle JSON columns - psycopg2 returns them as Python objects (list/dict)
        # or as JSON strings depending on connection settings
        def parse_json_field(field):
            if field is None:
                return []
            if isinstance(field, list):
                return field
            if isinstance(field, str):
                return json.loads(field)
            return []

        return User(
            id=str(user_id),
            email=email,
            password_hash=password_hash,
            name=name,
            programming_backgrounds=parse_json_field(programming_backgrounds_json),
            frameworks_known=parse_json_field(frameworks_known_json),
            hardware_experience=parse_json_field(hardware_experience_json),
            robotics_interest=robotics_interest,
            experience_level=experience_level,
            completed_onboarding=completed_onboarding,
            created_at=created_at,
            updated_at=updated_at
        )
