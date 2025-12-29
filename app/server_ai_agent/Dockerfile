# Use Python 3.11 slim image as base
FROM python:3.11-slim

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# Set work directory
WORKDIR /app

# Install system dependencies including Docker
RUN apt-get update && apt-get install -y \
    curl \
    build-essential \
    ca-certificates \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add Docker's official GPG key and repository
RUN install -m 0755 -d /etc/apt/keyrings && \
    curl -fsSL https://download.docker.com/linux/debian/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg && \
    chmod a+r /etc/apt/keyrings/docker.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker CLI
RUN apt-get update && apt-get install -y docker-ce-cli && rm -rf /var/lib/apt/lists/*

# Install UV package manager and ensure it's in PATH
RUN curl -LsSf https://astral.sh/uv/install.sh | sh && \
    mv ~/.cargo/bin/uv /usr/local/bin/uv || \
    mv ~/.local/bin/uv /usr/local/bin/uv || \
    ln -s ~/.cargo/bin/uv /usr/local/bin/uv 2>/dev/null || true

# Verify UV installation
RUN uv --version

# Copy dependency files (required by pyproject.toml)
COPY pyproject.toml uv.lock ./

# Install Python dependencies using UV
RUN uv sync

# Copy source code
COPY src/ ./src/
COPY .env.example ./

# Create entrypoint script that handles Docker socket permissions
RUN echo '#!/bin/bash\n\
# Get the docker group ID from the host\n\
DOCKER_GID=$(stat -c "%g" /var/run/docker.sock 2>/dev/null || echo "999")\n\
\n\
# Create docker group with the same GID as host\n\
groupadd -g $DOCKER_GID docker 2>/dev/null || true\n\
\n\
# Add appuser to docker group\n\
usermod -aG docker appuser 2>/dev/null || true\n\
\n\
# Switch to appuser and run the application\n\
exec gosu appuser "$@"' > /entrypoint.sh && \
chmod +x /entrypoint.sh

# Install gosu for user switching
RUN apt-get update && apt-get install -y gosu && rm -rf /var/lib/apt/lists/*

# Create a non-root user for security
RUN adduser --disabled-password --gecos '' appuser && \
    chown -R appuser:appuser /app

# Expose the port that the application runs on
EXPOSE 8089

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8089/health || exit 1

# Set default environment variables
ENV BRIDGE_HOST=0.0.0.0 \
    BRIDGE_PORT=8089 \
    CORS_ORIGINS="*"

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Command to run the application
CMD ["uv", "run", "mcp-bridge"]