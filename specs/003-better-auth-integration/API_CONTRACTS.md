# Authentication API Contracts

**Version**: 1.0.0
**Base URL**: `/api`
**Authentication**: JWT Bearer tokens in `Authorization` header

---

## Endpoints Overview

| Method | Endpoint | Purpose | Auth Required |
|--------|----------|---------|---|
| POST | `/auth/signup` | Register new user | No |
| POST | `/auth/signin` | Authenticate user | No |
| POST | `/auth/signout` | Logout user | Yes |
| GET | `/auth/session` | Get current user | Yes |
| GET | `/users/profile` | Get user profile | Yes |
| PUT | `/users/profile` | Update user profile | Yes |

---

## Authentication Request/Response Format

### Authorization Header Format
```
Authorization: Bearer <JWT_TOKEN>
```

### JWT Token Structure
```
Header: {
  "alg": "HS256",
  "typ": "JWT"
}

Payload: {
  "sub": "user-uuid",
  "iat": 1703001200,
  "exp": 1703087600
}

Signature: HMACSHA256(base64UrlEncode(header) + "." + base64UrlEncode(payload), AUTH_SECRET)
```

**Token Lifetime**: 24 hours

---

## Endpoint Details

### 1. POST `/auth/signup` - Register New User

**Purpose**: Create a new user account with profile information

**Request**:
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123",
  "name": "John Doe",
  "programming_backgrounds": ["Python", "JavaScript"],
  "frameworks_known": ["React", "FastAPI"],
  "hardware_experience": ["Arduino", "Raspberry Pi"],
  "robotics_interest": "Humanoid robotics",
  "experience_level": "beginner"
}
```

**Request Fields**:
- `email` (string, required): Valid email address (must be unique)
- `password` (string, required): Minimum 8 characters
- `name` (string, required): Full name (1-255 characters)
- `programming_backgrounds` (array, optional): Programming languages
- `frameworks_known` (array, optional): Web/app frameworks
- `hardware_experience` (array, optional): Hardware platforms
- `robotics_interest` (string, optional): Description of robotics interest
- `experience_level` (enum, default: "beginner"): One of "beginner", "intermediate", "advanced"

**Response** (201 Created):
```json
{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "user@example.com",
    "name": "John Doe",
    "programming_backgrounds": ["Python", "JavaScript"],
    "frameworks_known": ["React", "FastAPI"],
    "hardware_experience": ["Arduino", "Raspberry Pi"],
    "robotics_interest": "Humanoid robotics",
    "experience_level": "beginner",
    "completed_onboarding": true,
    "created_at": "2025-12-18T10:00:00Z",
    "updated_at": "2025-12-18T10:00:00Z"
  },
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "expires_at": "2025-12-19T10:00:00Z"
}
```

**Errors**:
- `400 Bad Request`: Validation failed
  ```json
  {
    "error": "Password must be at least 8 characters",
    "code": "VALIDATION_ERROR"
  }
  ```

- `409 Conflict`: Email already exists
  ```json
  {
    "error": "An account with this email already exists",
    "code": "USER_ALREADY_EXISTS"
  }
  ```

- `500 Internal Server Error`: Server error
  ```json
  {
    "error": "An error occurred during signup",
    "code": "SERVER_ERROR"
  }
  ```

---

### 2. POST `/auth/signin` - Authenticate User

**Purpose**: Authenticate user and receive JWT token

**Request**:
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123"
}
```

**Request Fields**:
- `email` (string, required): User email
- `password` (string, required): User password

**Response** (200 OK):
```json
{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "user@example.com",
    "name": "John Doe",
    "programming_backgrounds": ["Python", "JavaScript"],
    "frameworks_known": ["React", "FastAPI"],
    "hardware_experience": ["Arduino", "Raspberry Pi"],
    "robotics_interest": "Humanoid robotics",
    "experience_level": "beginner",
    "completed_onboarding": true,
    "created_at": "2025-12-18T10:00:00Z",
    "updated_at": "2025-12-18T10:00:00Z"
  },
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "expires_at": "2025-12-19T10:00:00Z"
}
```

**Errors**:
- `401 Unauthorized`: Invalid credentials (email or password incorrect)
  ```json
  {
    "error": "Invalid email or password",
    "code": "INVALID_CREDENTIALS"
  }
  ```

- `500 Internal Server Error`: Server error
  ```json
  {
    "error": "An error occurred during signin",
    "code": "SERVER_ERROR"
  }
  ```

**Note**: Error message is the same for missing user or incorrect password to prevent email enumeration.

---

### 3. POST `/auth/signout` - Logout User

**Purpose**: Invalidate user session (logout)

**Authentication**: Required (Bearer token)

**Request**: No request body required

**Response** (200 OK):
```json
{
  "success": true,
  "message": "Logged out successfully"
}
```

**Errors**:
- `401 Unauthorized`: No valid token provided
  ```json
  {
    "error": "Authentication required",
    "code": "UNAUTHORIZED"
  }
  ```

- `500 Internal Server Error`: Server error

**Note**: Client should delete token from localStorage/cookies after successful signout.

---

### 4. GET `/auth/session` - Get Current Session

**Purpose**: Retrieve current authenticated user's information

**Authentication**: Required (Bearer token)

**Request**: No request body

**Response** (200 OK):
```json
{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "user@example.com",
    "name": "John Doe",
    "programming_backgrounds": ["Python", "JavaScript"],
    "frameworks_known": ["React", "FastAPI"],
    "hardware_experience": ["Arduino", "Raspberry Pi"],
    "robotics_interest": "Humanoid robotics",
    "experience_level": "beginner",
    "completed_onboarding": true,
    "created_at": "2025-12-18T10:00:00Z",
    "updated_at": "2025-12-18T10:00:00Z"
  }
}
```

**Errors**:
- `401 Unauthorized`: No valid token or token expired
  ```json
  {
    "error": "Invalid or expired token",
    "code": "UNAUTHORIZED"
  }
  ```

- `404 Not Found`: User not found (deleted)
  ```json
  {
    "error": "User not found",
    "code": "USER_NOT_FOUND"
  }
  ```

**Use Case**: Frontend calls this on app load to restore user session

---

### 5. GET `/users/profile` - Get User Profile

**Purpose**: Retrieve full user profile information

**Authentication**: Required (Bearer token)

**Request**: No request body

**Response** (200 OK):
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "user@example.com",
  "name": "John Doe",
  "programming_backgrounds": ["Python", "JavaScript"],
  "frameworks_known": ["React", "FastAPI"],
  "hardware_experience": ["Arduino", "Raspberry Pi"],
  "robotics_interest": "Humanoid robotics",
  "experience_level": "beginner",
  "completed_onboarding": true,
  "created_at": "2025-12-18T10:00:00Z",
  "updated_at": "2025-12-18T10:00:00Z"
}
```

**Errors**:
- `401 Unauthorized`: No valid token
- `404 Not Found`: User not found
- `500 Internal Server Error`: Server error

---

### 6. PUT `/users/profile` - Update User Profile

**Purpose**: Update user profile information (any field can be updated independently)

**Authentication**: Required (Bearer token)

**Request** (all fields optional):
```json
{
  "name": "Jane Doe",
  "programming_backgrounds": ["Python", "JavaScript", "Rust"],
  "frameworks_known": ["React", "FastAPI", "Django"],
  "hardware_experience": ["Arduino", "Raspberry Pi", "Jetson"],
  "robotics_interest": "Bipedal locomotion",
  "experience_level": "intermediate"
}
```

**Request Fields** (all optional):
- `name` (string): Updated full name
- `programming_backgrounds` (array): Updated programming backgrounds
- `frameworks_known` (array): Updated frameworks known
- `hardware_experience` (array): Updated hardware experience
- `robotics_interest` (string): Updated robotics interest
- `experience_level` (enum): One of "beginner", "intermediate", "advanced"

**Response** (200 OK):
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "user@example.com",
  "name": "Jane Doe",
  "programming_backgrounds": ["Python", "JavaScript", "Rust"],
  "frameworks_known": ["React", "FastAPI", "Django"],
  "hardware_experience": ["Arduino", "Raspberry Pi", "Jetson"],
  "robotics_interest": "Bipedal locomotion",
  "experience_level": "intermediate",
  "completed_onboarding": true,
  "created_at": "2025-12-18T10:00:00Z",
  "updated_at": "2025-12-18T10:30:00Z"
}
```

**Errors**:
- `400 Bad Request`: Invalid experience_level value
  ```json
  {
    "error": "Experience level must be one of: beginner, intermediate, advanced",
    "code": "VALIDATION_ERROR"
  }
  ```

- `401 Unauthorized`: No valid token
- `404 Not Found`: User not found
- `500 Internal Server Error`: Server error

**Note**: Only provided fields are updated; omitted fields are left unchanged.

---

## Error Response Format

All errors follow this format:
```json
{
  "error": "Human-readable error message",
  "code": "ERROR_CODE",
  "details": "Optional additional details"
}
```

### Error Codes:
- `VALIDATION_ERROR` - Input validation failed
- `INVALID_CREDENTIALS` - Login failed (wrong password/email)
- `USER_ALREADY_EXISTS` - Email already registered
- `USER_NOT_FOUND` - User doesn't exist
- `UNAUTHORIZED` - Not authenticated or token invalid
- `SERVER_ERROR` - Unexpected server error

---

## HTTP Status Codes

| Code | Meaning |
|------|---------|
| 200 | OK - Request succeeded |
| 201 | Created - Resource created successfully |
| 400 | Bad Request - Validation or client error |
| 401 | Unauthorized - Authentication required or failed |
| 404 | Not Found - Resource not found |
| 409 | Conflict - Resource already exists (e.g., duplicate email) |
| 500 | Internal Server Error - Server-side error |

---

## CORS Headers

All endpoints support CORS with the following headers (if frontend is on allowed origin):
```
Access-Control-Allow-Origin: <frontend-url>
Access-Control-Allow-Credentials: true
Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS
Access-Control-Allow-Headers: *
```

**Note**: Credentials must be allowed for cookies to work properly.

---

## Frontend Integration Example

### TypeScript/React Example

```typescript
// 1. Signup
const signup = async (email: string, password: string, backgroundData: any) => {
  const response = await fetch('/api/auth/signup', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      email,
      password,
      ...backgroundData
    }),
    credentials: 'include' // Important for CORS
  });

  const data = await response.json();

  if (response.ok) {
    // Store token
    localStorage.setItem('auth_token', data.token);
    return data.user;
  } else {
    throw new Error(data.error);
  }
};

// 2. Signin
const signin = async (email: string, password: string) => {
  const response = await fetch('/api/auth/signin', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email, password }),
    credentials: 'include'
  });

  const data = await response.json();

  if (response.ok) {
    localStorage.setItem('auth_token', data.token);
    return data.user;
  } else {
    throw new Error(data.error);
  }
};

// 3. Check session (call on app load)
const checkSession = async () => {
  const token = localStorage.getItem('auth_token');
  if (!token) return null;

  const response = await fetch('/api/auth/session', {
    method: 'GET',
    headers: {
      'Authorization': `Bearer ${token}`,
      'Content-Type': 'application/json'
    },
    credentials: 'include'
  });

  const data = await response.json();

  if (response.ok) {
    return data.user;
  } else {
    localStorage.removeItem('auth_token');
    return null;
  }
};

// 4. API client with auth headers
const apiClient = async (endpoint: string, options: RequestInit = {}) => {
  const token = localStorage.getItem('auth_token');

  const headers = {
    'Content-Type': 'application/json',
    ...options.headers,
  };

  if (token) {
    headers['Authorization'] = `Bearer ${token}`;
  }

  const response = await fetch(`/api${endpoint}`, {
    ...options,
    headers,
    credentials: 'include'
  });

  const data = await response.json();

  if (response.status === 401) {
    localStorage.removeItem('auth_token');
    // Redirect to signin
    window.location.href = '/signin';
  }

  return { ok: response.ok, status: response.status, data };
};

// 5. Get profile
const getProfile = () => apiClient('/users/profile', { method: 'GET' });

// 6. Update profile
const updateProfile = (updates: any) =>
  apiClient('/users/profile', {
    method: 'PUT',
    body: JSON.stringify(updates)
  });

// 7. Logout
const logout = async () => {
  const token = localStorage.getItem('auth_token');
  if (token) {
    await fetch('/api/auth/signout', {
      method: 'POST',
      headers: { 'Authorization': `Bearer ${token}` },
      credentials: 'include'
    });
  }
  localStorage.removeItem('auth_token');
  window.location.href = '/';
};
```

---

## Field Definitions

### Programming Backgrounds (Examples)
- Python
- JavaScript / TypeScript
- C / C++
- Java
- C#
- Go
- Rust
- PHP
- Ruby
- Swift
- Kotlin
- Scala

### Frameworks Known (Examples)
- React
- Vue
- Angular
- Next.js
- Svelte
- Django
- FastAPI
- Flask
- Express
- Spring Boot
- Rails
- ASP.NET

### Hardware Experience (Examples)
- Arduino
- Raspberry Pi
- NVIDIA Jetson
- Microcontroller
- Sensors & Actuators
- Robot Kits (VEX, LEGO, etc.)
- FPGAs
- IoT Devices
- No Experience

### Experience Levels
- **beginner**: New to robotics/AI, learning fundamentals
- **intermediate**: Has worked on some projects, understands basic concepts
- **advanced**: Experienced with production systems, deep domain knowledge

---

## Rate Limiting

**Current**: Not implemented (add in production)

**Recommended** (for future):
- 5 failed login attempts per IP per minute → 429 Too Many Requests
- 10 signup attempts per IP per hour → 429 Too Many Requests
- General: 100 requests per minute per authenticated user → 429 Too Many Requests

---

## Security Headers

Recommended security headers (add to CORS/app config):
```
X-Content-Type-Options: nosniff
X-Frame-Options: DENY
X-XSS-Protection: 1; mode=block
Strict-Transport-Security: max-age=31536000; includeSubDomains
Content-Security-Policy: default-src 'self'
```

---

## Deployment Checklist

Before deploying to production:

- [ ] `AUTH_SECRET` set to random 32+ character string
- [ ] `DATABASE_URL` points to production database
- [ ] Database migration script applied
- [ ] `CORS_ORIGINS` updated with production frontend URL
- [ ] HTTPS enforced (redirect HTTP to HTTPS)
- [ ] Security headers configured
- [ ] Database backups configured
- [ ] Monitoring and alerting set up for auth failures
- [ ] Rate limiting implemented
- [ ] Secrets not committed to Git

