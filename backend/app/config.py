from pydantic_settings import BaseSettings
from typing import List, Optional

class Settings(BaseSettings):
    # API Settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "Physical AI & Humanoid Robotics RAG API"
    
    # Database
    DATABASE_URL: str = "sqlite:///./rag_chatbot.db"  # Default, can be overridden
    
    # Vector Database (Qdrant)
    QDRANT_URL: str
    QDRANT_API_KEY: str
    
    # LLM Settings
    GROQ_API_KEY: str
    GROQ_MODEL_NAME: str = "llama3-8b-8192"  # Default model, can be overridden to llama3-70b-8192
    
    # Application
    DEBUG: bool = False
    
    class Config:
        env_file = ".env"

settings = Settings()