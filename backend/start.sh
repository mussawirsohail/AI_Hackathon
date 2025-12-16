#!/bin/bash
# Start the FastAPI application with uvicorn

echo "Starting Physical AI & Humanoid Robotics RAG API..."
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload