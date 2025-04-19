"""
Unit tests for the authentication API endpoints

These tests verify the functionality of the authentication API,
ensuring proper user login, token generation, and role-based access control.
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock

from run import app
from service.auth_service import (
    User, UserInDB, Token, UserRole,
    fake_users_db, get_password_hash
)


@pytest.fixture
def client():
    """Create a test client for the FastAPI application"""
    return TestClient(app)


def test_login_for_access_token(client):
    """Test successful user login and token generation"""
    response = client.post(
        "/api/v1/auth/token",
        data={"username": "admin", "password": "admin"}
    )
    assert response.status_code == 200
    
    token_data = response.json()
    assert token_data["token_type"] == "bearer"
    assert "access_token" in token_data
    assert "expires_at" in token_data
    assert "user" in token_data
    assert token_data["user"]["username"] == "admin"
    assert token_data["user"]["role"] == "admin"


def test_login_with_invalid_credentials(client):
    """Test login with invalid credentials"""
    response = client.post(
        "/api/v1/auth/token",
        data={"username": "admin", "password": "wrong_password"}
    )
    assert response.status_code == 401
    assert "detail" in response.json()
    assert response.json()["detail"] == "Incorrect username or password"


def test_read_users_me(client):
    """Test getting current user information"""
    # First login to get a token
    login_response = client.post(
        "/api/v1/auth/token",
        data={"username": "admin", "password": "admin"}
    )
    token = login_response.json()["access_token"]
    
    # Use token to get user info
    response = client.get(
        "/api/v1/auth/me",
        headers={"Authorization": f"Bearer {token}"}
    )
    assert response.status_code == 200
    
    user_data = response.json()
    assert user_data["username"] == "admin"
    assert user_data["role"] == "admin"


def test_read_users_me_without_token(client):
    """Test getting current user without authentication"""
    response = client.get("/api/v1/auth/me")
    assert response.status_code == 401
    assert "detail" in response.json()


def test_read_users(client):
    """Test getting all users (admin only)"""
    # Login as admin to get a token
    login_response = client.post(
        "/api/v1/auth/token",
        data={"username": "admin", "password": "admin"}
    )
    admin_token = login_response.json()["access_token"]
    
    # Use admin token to get users
    response = client.get(
        "/api/v1/auth/users",
        headers={"Authorization": f"Bearer {admin_token}"}
    )
    assert response.status_code == 200
    
    users = response.json()
    assert len(users) >= 3  # At least admin, operator, and viewer
    
    # Verify expected users are in the list
    usernames = [user["username"] for user in users]
    assert "admin" in usernames
    assert "operator" in usernames
    assert "viewer" in usernames


def test_read_users_as_non_admin(client):
    """Test getting all users as non-admin (should be forbidden)"""
    # Login as viewer to get a token
    login_response = client.post(
        "/api/v1/auth/token",
        data={"username": "viewer", "password": "viewer"}
    )
    viewer_token = login_response.json()["access_token"]
    
    # Try to get users as viewer
    response = client.get(
        "/api/v1/auth/users",
        headers={"Authorization": f"Bearer {viewer_token}"}
    )
    assert response.status_code == 403  # Forbidden


def test_create_user(client):
    """Test creating a new user (admin only)"""
    # Login as admin to get a token
    login_response = client.post(
        "/api/v1/auth/token",
        data={"username": "admin", "password": "admin"}
    )
    admin_token = login_response.json()["access_token"]
    
    # Create a test user
    test_user = {
        "username": "testuser",
        "password": "testpassword",
        "email": "test@example.com",
        "full_name": "Test User",
        "role": "viewer"
    }
    
    response = client.post(
        "/api/v1/auth/users",
        params=test_user,
        headers={"Authorization": f"Bearer {admin_token}"}
    )
    assert response.status_code == 200
    
    created_user = response.json()
    assert created_user["username"] == "testuser"
    assert created_user["email"] == "test@example.com"
    assert created_user["role"] == "viewer"
    
    # Verify user was added to the database
    assert "testuser" in fake_users_db


def test_create_user_as_non_admin(client):
    """Test creating a new user as non-admin (should be forbidden)"""
    # Login as viewer to get a token
    login_response = client.post(
        "/api/v1/auth/token",
        data={"username": "viewer", "password": "viewer"}
    )
    viewer_token = login_response.json()["access_token"]
    
    # Try to create a user as viewer
    test_user = {
        "username": "testuser2",
        "password": "testpassword",
        "email": "test2@example.com",
        "full_name": "Test User 2",
        "role": "viewer"
    }
    
    response = client.post(
        "/api/v1/auth/users",
        params=test_user,
        headers={"Authorization": f"Bearer {viewer_token}"}
    )
    assert response.status_code == 403  # Forbidden


def test_disable_and_enable_user(client):
    """Test disabling and enabling a user (admin only)"""
    # Login as admin to get a token
    login_response = client.post(
        "/api/v1/auth/token",
        data={"username": "admin", "password": "admin"}
    )
    admin_token = login_response.json()["access_token"]
    
    # First create a test user
    test_user = {
        "username": "disable_test",
        "password": "testpassword",
        "email": "disable@example.com",
        "full_name": "Disable Test",
        "role": "viewer"
    }
    
    client.post(
        "/api/v1/auth/users",
        params=test_user,
        headers={"Authorization": f"Bearer {admin_token}"}
    )
    
    # Disable the user
    disable_response = client.put(
        "/api/v1/auth/users/disable_test/disable",
        headers={"Authorization": f"Bearer {admin_token}"}
    )
    assert disable_response.status_code == 200
    
    disabled_user = disable_response.json()
    assert disabled_user["disabled"] == True
    
    # Enable the user
    enable_response = client.put(
        "/api/v1/auth/users/disable_test/enable",
        headers={"Authorization": f"Bearer {admin_token}"}
    )
    assert enable_response.status_code == 200
    
    enabled_user = enable_response.json()
    assert enabled_user["disabled"] == False