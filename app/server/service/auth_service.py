"""
Authentication Service for WiseVision Dashboard

This module provides authentication and authorization services for the WiseVision Dashboard,
implementing Role-Based Access Control (RBAC) with JWT token-based authentication.

Roles:
- Admin: Full access to all features, including sending commands to robots
- Operator: Can view data and send limited commands but cannot change system configuration
- Viewer: Read-only access to the dashboard
"""

import os
import time
from datetime import datetime, timedelta
from enum import Enum
from typing import Dict, List, Optional, Union

import jwt
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from passlib.context import CryptContext
from pydantic import BaseModel

# Configure JWT
SECRET_KEY = os.environ.get("JWT_SECRET_KEY", "wisevision-dashboard-dev-key")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = int(os.environ.get("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

# Configure password hashing
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# OAuth2 scheme for token retrieval
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="api/v1/auth/token")


class UserRole(str, Enum):
    """User roles for Role-Based Access Control"""
    ADMIN = "admin"
    OPERATOR = "operator"
    VIEWER = "viewer"


class User(BaseModel):
    """User model"""
    username: str
    email: Optional[str] = None
    full_name: Optional[str] = None
    disabled: Optional[bool] = False
    role: UserRole


class UserInDB(User):
    """User model with hashed password for database storage"""
    hashed_password: str


class Token(BaseModel):
    """Token response model"""
    access_token: str
    token_type: str
    expires_at: datetime
    user: Dict


class TokenData(BaseModel):
    """Token payload model"""
    username: Optional[str] = None
    role: Optional[str] = None


# In-memory user database for development/testing
# In production, this would be replaced with a database service
fake_users_db = {
    "admin": {
        "username": "admin",
        "full_name": "Administrator",
        "email": "admin@wisevision.example",
        "hashed_password": pwd_context.hash("admin"),
        "disabled": False,
        "role": UserRole.ADMIN
    },
    "operator": {
        "username": "operator",
        "full_name": "System Operator",
        "email": "operator@wisevision.example",
        "hashed_password": pwd_context.hash("operator"),
        "disabled": False,
        "role": UserRole.OPERATOR
    },
    "viewer": {
        "username": "viewer",
        "full_name": "Dashboard Viewer",
        "email": "viewer@wisevision.example",
        "hashed_password": pwd_context.hash("viewer"),
        "disabled": False,
        "role": UserRole.VIEWER
    }
}


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against a hash"""
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """Generate a hash from a password"""
    return pwd_context.hash(password)


def get_user(db, username: str) -> Optional[UserInDB]:
    """Get a user from the database by username"""
    if username in db:
        user_dict = db[username]
        return UserInDB(**user_dict)
    return None


def authenticate_user(db, username: str, password: str) -> Union[User, bool]:
    """Authenticate a user with username and password"""
    user = get_user(db, username)
    if not user:
        return False
    if not verify_password(password, user.hashed_password):
        return False
    return user


def create_access_token(
    data: dict, 
    expires_delta: Optional[timedelta] = None
) -> str:
    """Create a JWT access token"""
    to_encode = data.copy()
    
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


async def get_current_user(token: str = Depends(oauth2_scheme)) -> User:
    """
    Get the current authenticated user from the token
    
    Raises:
        HTTPException: If the token is invalid or the user is not found
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    
    try:
        # Decode the JWT token
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        username: str = payload.get("sub")
        role: str = payload.get("role")
        
        if username is None:
            raise credentials_exception
        
        token_data = TokenData(username=username, role=role)
    except jwt.PyJWTError:
        raise credentials_exception
    
    user = get_user(fake_users_db, username=token_data.username)
    if user is None:
        raise credentials_exception
    
    return User(
        username=user.username,
        email=user.email,
        full_name=user.full_name,
        disabled=user.disabled,
        role=user.role
    )


async def get_current_active_user(
    current_user: User = Depends(get_current_user)
) -> User:
    """
    Get the current active (non-disabled) user
    
    Raises:
        HTTPException: If the user is disabled
    """
    if current_user.disabled:
        raise HTTPException(status_code=400, detail="Inactive user")
    return current_user


def has_role(required_roles: List[UserRole]):
    """
    Dependency for role-based access control
    
    Args:
        required_roles: List of roles that are allowed to access the endpoint
        
    Returns:
        A dependency function that checks if the user has one of the required roles
    """
    async def role_checker(current_user: User = Depends(get_current_active_user)):
        if current_user.role not in required_roles:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail=f"Insufficient permissions. Required roles: {', '.join([r.value for r in required_roles])}"
            )
        return current_user
    return role_checker


# Predefined role-based access dependencies
admin_only = has_role([UserRole.ADMIN])
admin_or_operator = has_role([UserRole.ADMIN, UserRole.OPERATOR])
any_role = has_role([UserRole.ADMIN, UserRole.OPERATOR, UserRole.VIEWER])