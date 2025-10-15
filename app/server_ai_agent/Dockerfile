# Use Python 3.11 slim image as base
FROM python:3.11-slim

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# Set work directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install UV package manager and ensure it's in PATH
RUN curl -LsSf https://astral.sh/uv/install.sh | sh && \
    mv ~/.cargo/bin/uv /usr/local/bin/uv || \
    mv ~/.local/bin/uv /usr/local/bin/uv || \
    ln -s ~/.cargo/bin/uv /usr/local/bin/uv 2>/dev/null || true

# Verify UV installation
RUN uv --version

# Copy dependency files and README (required by pyproject.toml)
COPY pyproject.toml uv.lock README.md ./

# Install Python dependencies using UV
RUN uv sync --frozen --no-dev

# Copy source code
COPY src/ ./src/
COPY start.sh ./
COPY .env.example ./

# Make start script executable
RUN chmod +x start.sh

# Create a non-root user for security
RUN adduser --disabled-password --gecos '' appuser && \
    chown -R appuser:appuser /app
USER appuser

# Expose the port that the application runs on
EXPOSE 8089

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8089/health || exit 1

# Set default environment variables
ENV BRIDGE_HOST=0.0.0.0 \
    BRIDGE_PORT=8089 \
    CORS_ORIGINS="*"

# Command to run the application
CMD ["uv", "run", "mcp-bridge"]