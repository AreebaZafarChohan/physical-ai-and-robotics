FROM python:3.10-slim

ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

WORKDIR /app

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        gcc \
    && rm -rf /var/lib/apt/lists/*

COPY backend/requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir --upgrade pip
RUN pip install --no-cache-dir -r requirements.txt

COPY backend/ /app/backend/

# HF ko 7860 chahiye
EXPOSE 7860

# 🔥 MAGIC LINE — code unchanged, port same
CMD ["sh", "-c", "uvicorn backend.src.api.main:app --host 0.0.0.0 --port ${PORT:-9000}"]
