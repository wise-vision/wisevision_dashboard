"""
Authentication API for WiseVision Dashboard

This module provides the authentication API endpoints for the WiseVision Dashboard,
handling user login, token generation, and user management.
"""

import logging
from datetime import datetime, timedelta
from typing import Dict, List

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordRequestForm

from ..service.auth_service import (
    User, UserInDB, Token, UserRole,
    fake_users_db, authenticate_user, create_access_token,
    get_current_active_user, admin_only, ACCESS_TOKEN_EXPIRE_MINUTES
)

# Set up logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter(prefix="/api/v1/auth", tags=["auth"])


@router.post("/token", response_model=Token)
async def login_for_access_token(form_data: OAuth2PasswordRequestForm = Depends()):
    """
    Authenticate user and generate access token
    
    Args:
        form_data: OAuth2 password request form
        
    Returns:
        Token: JWT access token, expiry, and user information
        
    Raises:
        HTTPException: If authentication fails
    """
    user = authenticate_user(fake_users_db, form_data.username, form_data.password)
    if not user:
        logger.warning(f"Failed login attempt for user: {form_data.username}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.username, "role": user.role},
        expires_delta=access_token_expires
    )
    
    logger.info(f"User {user.username} authenticated successfully")
    
    # Calculate expiry time for client convenience
    expires_at = datetime.utcnow() + access_token_expires
    
    return Token(
        access_token=access_token,
        token_type="bearer",
        expires_at=expires_at,
        user={
            "username": user.username,
            "role": user.role,
            "full_name": user.full_name,
            "email": user.email
        }
    )


@router.get("/me", response_model=User)
async def read_users_me(current_user: User = Depends(get_current_active_user)):
    """
    Get the current user's information
    
    Args:
        current_user: Current authenticated user
        
    Returns:
        User: Current user information
    """
    return current_user


@router.get("/users", response_model=List[User])
async def read_users(current_user: User = Depends(admin_only)):
    """
    Get list of users (admin only)
    
    Args:
        current_user: Current authenticated admin user
        
    Returns:
        List[User]: List of users
    """
    users = []
    for username, user_data in fake_users_db.items():
        users.append(User(
            username=username,
            email=user_data["email"],
            full_name=user_data["full_name"],
            disabled=user_data["disabled"],
            role=user_data["role"]
        ))
    return users


@router.post("/users", response_model=User)
async def create_user(
    username: str,
    password: str,
    email: str,
    full_name: str,
    role: UserRole,
    current_user: User = Depends(admin_only)
):
    """
    Create a new user (admin only)
    
    Args:
        username: Username for the new user
        password: Password for the new user
        email: Email for the new user
        full_name: Full name for the new user
        role: Role for the new user
        current_user: Current authenticated admin user
        
    Returns:
        User: Created user information
        
    Raises:
        HTTPException: If username already exists
    """
    if username in fake_users_db:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Username already exists"
        )
    
    from ..service.auth_service import get_password_hash
    
    user_dict = {
        "username": username,
        "email": email,
        "full_name": full_name,
        "disabled": False,
        "role": role,
        "hashed_password": get_password_hash(password)
    }
    
    fake_users_db[username] = user_dict
    
    logger.info(f"User {username} created by {current_user.username}")
    
    return User(
        username=username,
        email=email,
        full_name=full_name,
        disabled=False,
        role=role
    )


@router.put("/users/{username}/disable", response_model=User)
async def disable_user(
    username: str,
    current_user: User = Depends(admin_only)
):
    """
    Disable a user (admin only)
    
    Args:
        username: Username of user to disable
        current_user: Current authenticated admin user
        
    Returns:
        User: Updated user information
        
    Raises:
        HTTPException: If username doesn't exist or is the current user
    """
    if username not in fake_users_db:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )
    
    if username == current_user.username:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Cannot disable your own account"
        )
    
    fake_users_db[username]["disabled"] = True
    
    logger.info(f"User {username} disabled by {current_user.username}")
    
    return User(
        username=username,
        email=fake_users_db[username]["email"],
        full_name=fake_users_db[username]["full_name"],
        disabled=True,
        role=fake_users_db[username]["role"]
    )


@router.put("/users/{username}/enable", response_model=User)
async def enable_user(
    username: str,
    current_user: User = Depends(admin_only)
):
    """
    Enable a disabled user (admin only)
    
    Args:
        username: Username of user to enable
        current_user: Current authenticated admin user
        
    Returns:
        User: Updated user information
        
    Raises:
        HTTPException: If username doesn't exist
    """
    if username not in fake_users_db:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )
    
    fake_users_db[username]["disabled"] = False
    
    logger.info(f"User {username} enabled by {current_user.username}")
    
    return User(
        username=username,
        email=fake_users_db[username]["email"],
        full_name=fake_users_db[username]["full_name"],
        disabled=False,
        role=fake_users_db[username]["role"]
    )