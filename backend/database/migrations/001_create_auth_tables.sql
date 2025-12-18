-- Migration: Create authentication tables
-- Date: 2025-12-18
-- Purpose: Set up users and user_profiles tables for Better Auth integration

-- Users table: core authentication information
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) NOT NULL UNIQUE,
    password_hash VARCHAR(255) NOT NULL,
    name VARCHAR(255),
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,

    -- Constraints
    CONSTRAINT email_not_empty CHECK (email != ''),
    CONSTRAINT name_not_empty CHECK (name IS NULL OR name != '')
);

-- Create indexes for users table
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_created_at ON users(created_at);

-- User profiles table: extended profile information with background data
CREATE TABLE IF NOT EXISTS user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,

    -- Background information (stored as JSON for flexibility)
    programming_backgrounds JSONB DEFAULT '[]'::JSONB,
    frameworks_known JSONB DEFAULT '[]'::JSONB,
    hardware_experience JSONB DEFAULT '[]'::JSONB,

    -- Experience and interests
    robotics_interest VARCHAR(255),
    experience_level VARCHAR(50) NOT NULL DEFAULT 'beginner'
        CHECK (experience_level IN ('beginner', 'intermediate', 'advanced')),

    -- Onboarding status
    completed_onboarding BOOLEAN NOT NULL DEFAULT FALSE,

    -- Metadata
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,

    -- Constraints
    CONSTRAINT experience_level_valid CHECK (experience_level IN ('beginner', 'intermediate', 'advanced'))
);

-- Create indexes for user_profiles table
CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
CREATE INDEX idx_user_profiles_experience_level ON user_profiles(experience_level);

-- Sessions table: for server-side session management (optional but recommended)
CREATE TABLE IF NOT EXISTS sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token VARCHAR(500) NOT NULL UNIQUE,
    expires_at TIMESTAMP NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,

    -- Indexes
    CONSTRAINT token_not_empty CHECK (token != '')
);

-- Create indexes for sessions table
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_token ON sessions(token);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);

-- Email verification tokens table (for future email verification feature)
CREATE TABLE IF NOT EXISTS email_verification_tokens (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token VARCHAR(255) NOT NULL UNIQUE,
    expires_at TIMESTAMP NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,

    -- Constraints
    CONSTRAINT token_not_empty CHECK (token != '')
);

-- Create indexes for email verification tokens
CREATE INDEX idx_email_verification_tokens_user_id ON email_verification_tokens(user_id);
CREATE INDEX idx_email_verification_tokens_token ON email_verification_tokens(token);
CREATE INDEX idx_email_verification_tokens_expires_at ON email_verification_tokens(expires_at);

-- Auth events log table: for security auditing
CREATE TABLE IF NOT EXISTS auth_events (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    event_type VARCHAR(50) NOT NULL
        CHECK (event_type IN ('signup', 'signin', 'signin_failed', 'signout', 'profile_updated', 'password_changed')),
    ip_address INET,
    user_agent VARCHAR(500),
    status VARCHAR(50) NOT NULL DEFAULT 'success'
        CHECK (status IN ('success', 'failed')),
    error_message TEXT,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for auth events table
CREATE INDEX idx_auth_events_user_id ON auth_events(user_id);
CREATE INDEX idx_auth_events_event_type ON auth_events(event_type);
CREATE INDEX idx_auth_events_created_at ON auth_events(created_at);

-- Comment on tables for documentation
COMMENT ON TABLE users IS 'Core user accounts table - stores authentication credentials and basic user info';
COMMENT ON TABLE user_profiles IS 'Extended user profile information including background, experience level, and interests';
COMMENT ON TABLE sessions IS 'Active user sessions for authentication state management';
COMMENT ON TABLE email_verification_tokens IS 'Temporary tokens for email verification (future feature)';
COMMENT ON TABLE auth_events IS 'Audit log of authentication events for security and debugging';

-- Comment on columns for documentation
COMMENT ON COLUMN users.id IS 'Unique user identifier (UUID)';
COMMENT ON COLUMN users.email IS 'User email address - used for login (unique)';
COMMENT ON COLUMN users.password_hash IS 'Hashed password using bcrypt (never store plaintext)';
COMMENT ON COLUMN user_profiles.programming_backgrounds IS 'Array of programming languages/backgrounds (JSON) - e.g., ["Python", "JavaScript", "C++"]';
COMMENT ON COLUMN user_profiles.frameworks_known IS 'Array of frameworks known (JSON) - e.g., ["React", "FastAPI", "ROS"]';
COMMENT ON COLUMN user_profiles.hardware_experience IS 'Array of hardware experience (JSON) - e.g., ["Arduino", "Raspberry Pi", "Jetson"]';
COMMENT ON COLUMN user_profiles.experience_level IS 'User experience level: beginner, intermediate, or advanced';
COMMENT ON COLUMN user_profiles.completed_onboarding IS 'Flag indicating if user completed the multi-step signup process';
COMMENT ON COLUMN auth_events.ip_address IS 'IP address from which auth event occurred (for security tracking)';
COMMENT ON COLUMN auth_events.user_agent IS 'User agent string (browser/client info) from auth event';
