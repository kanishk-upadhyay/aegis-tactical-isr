# ==============================================================================
# AEGIS-TACTICAL-ISR: Production Edge Container Specification
# Multi-stage security-hardened container for tactical deployment & edge nodes
# ==============================================================================

# Stage 1: Build & Dependencies
FROM python:3.11-slim-bookworm AS builder

WORKDIR /build

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir --user -r requirements.txt

# Stage 2: Runtime Environment
FROM python:3.11-slim-bookworm AS runner

LABEL maintainer="Army Design Bureau / DG AAD Internship Project" \
      system="AEGIS-TACTICAL-ISR" \
      version="1.0.0"

WORKDIR /app

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PATH=/home/aegis/.local/bin:$PATH \
    PYTHONPATH=/app

# Create unprivileged tactical operator user
RUN groupadd -g 10001 aegis && \
    useradd -u 10001 -g aegis -m -s /bin/bash aegis

# Copy python dependencies from builder
COPY --from=builder /root/.local /home/aegis/.local

# Copy application codebase
COPY --chown=aegis:aegis . /app

USER aegis

EXPOSE 8000

# Edge Node Healthcheck
HEALTHCHECK --interval=15s --timeout=5s --start-period=5s --retries=3 \
  CMD python3 -c "import urllib.request; urllib.request.urlopen('http://localhost:8000/api/v1/health')" || exit 1

ENTRYPOINT ["uvicorn", "server.app:app", "--host", "0.0.0.0", "--port", "8000"]
