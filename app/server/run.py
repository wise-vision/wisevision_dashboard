"""
WiseVision Dashboard FastAPI Application

This is the main entry point for the WiseVision Dashboard backend.
It configures and starts the FastAPI application with all routes and middleware.
"""

import logging
import os
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

# Import API routers
from api.web.auth_api import router as auth_router
from api.web.messages_api import router as messages_router
from api.web.openai_api import router as openai_router

# Import services
from service.ros2_manager import ROS2Manager
from service.data_black_box_client import DataBlackBoxClient
from service.openai_service import OpenAIService

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(),  # Log to console
        logging.FileHandler("wisevision_dashboard.log")  # Log to file
    ]
)
logger = logging.getLogger("wisevision_dashboard")


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Handle application startup and shutdown events
    
    This function initializes all necessary services when the app starts up
    and cleans up resources when the app shuts down.
    """
    # Startup: Initialize services
    logger.info("Starting WiseVision Dashboard API server")
    
    # Initialize the ROS 2 manager
    ros2_manager = ROS2Manager()
    logger.info("ROS 2 Manager initialized")
    
    # Initialize the Data Black Box client
    data_black_box_client = DataBlackBoxClient()
    logger.info("Data Black Box Client initialized")
    
    # Initialize the OpenAI service if API key is provided
    if os.environ.get("OPENAI_API_KEY"):
        openai_service = OpenAIService()
        logger.info("OpenAI Service initialized")
    else:
        logger.warning("OpenAI API key not provided. Natural language command processing will not be available.")
    
    yield  # Application runs here
    
    # Shutdown: Clean up resources
    logger.info("Shutting down WiseVision Dashboard API server")
    
    # Shutdown ROS 2 manager
    ros2_manager.shutdown()
    logger.info("ROS 2 Manager shut down")


# Create FastAPI application
app = FastAPI(
    title="WiseVision Dashboard API",
    description="API for the WiseVision Dashboard, providing access to ROS 2 topics, services, and actions",
    version="1.0.0",
    lifespan=lifespan
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=os.environ.get("CORS_ORIGINS", "http://localhost:3000").split(","),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(auth_router)
app.include_router(messages_router)
app.include_router(openai_router)

# Global exception handler
@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """Global exception handler for uncaught exceptions"""
    logger.error(f"Uncaught exception: {exc}", exc_info=True)
    if isinstance(exc, HTTPException):
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.detail}
        )
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal server error"}
    )

# Root endpoint
@app.get("/")
async def root():
    """Root endpoint providing basic API information"""
    return {
        "name": "WiseVision Dashboard API",
        "version": "1.0.0",
        "status": "online"
    }

# Health check endpoint
@app.get("/health")
async def health():
    """Health check endpoint"""
    try:
        # Check services health
        ros2_manager = ROS2Manager()
        ros2_status = ros2_manager.check_health()
        
        data_black_box_client = DataBlackBoxClient()
        data_black_box_status = data_black_box_client.check_health()
        
        openai_status = {"status": "disabled"}
        if os.environ.get("OPENAI_API_KEY"):
            openai_status = {"status": "ok"}
        
        return {
            "status": "ok",
            "services": {
                "api": {"status": "ok"},
                "ros2": ros2_status,
                "data_black_box": data_black_box_status,
                "openai": openai_status
            }
        }
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return {
            "status": "error",
            "message": str(e)
        }

# Serve static files (if any)
try:
    app.mount("/static", StaticFiles(directory="static"), name="static")
except RuntimeError:
    logger.warning("Static directory not found, skipping static file serving")

if __name__ == "__main__":
    # Get configuration from environment variables with defaults
    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "8000"))
    log_level = os.environ.get("LOG_LEVEL", "info")
    
    # Run the application
    uvicorn.run(
        "run:app",
        host=host,
        port=port,
        log_level=log_level,
        reload=os.environ.get("DEBUG", "False").lower() == "true"
    )