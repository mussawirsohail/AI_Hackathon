---
sidebar_position: 4
---

# Lesson 4: Building the RAG Backend with FastAPI

## Introduction

In this lesson, we'll build the core backend functionality for our RAG chatbot using FastAPI. This includes implementing the retrieval logic that fetches relevant documents from our vector database and the generation logic that creates responses based on the retrieved context.

## FastAPI Application Structure

Let's start with the main application setup:

```python
# backend/app/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.v1.api import api_router
from app.config import settings
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="RAG Chatbot API",
    description="API for Retrieval-Augmented Generation chatbot",
    version="1.0.0",
    openapi_url="/api/v1/openapi.json"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose headers for authentication
    expose_headers=["Access-Control-Allow-Origin"]
)

# Include API routes
app.include_router(api_router, prefix="/api/v1")

@app.on_event("startup")
async def startup_event():
    """Initialize resources on startup"""
    logger.info("Starting up RAG Chatbot API")
    # Initialize vector database connection
    # Initialize LLM clients
    # Load any necessary models

@app.on_event("shutdown")
async def shutdown_event():
    """Clean up resources on shutdown"""
    logger.info("Shutting down RAG Chatbot API")
    # Close database connections
    # Clean up resources

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}
```

## RAG Core Service

Now let's implement the core RAG service that orchestrates retrieval and generation:

```python
# backend/app/services/rag_service.py
import asyncio
from typing import List, Dict, Any, Optional
from app.vector_db import VectorDBManager
from app.document_processor import DocumentProcessor
from app.config import settings
from app.models.user import UserRead
import openai
import logging

class RAGService:
    def __init__(self):
        self.vector_db = VectorDBManager(
            url=settings.QDRANT_URL, 
            api_key=settings.QDRANT_API_KEY
        )
        self.doc_processor = DocumentProcessor()
        self.openai_client = openai.AsyncOpenAI(api_key=settings.OPENAI_API_KEY)
        
    async def generate_response(
        self, 
        query: str, 
        user_background: Optional[Dict[str, str]] = None,
        user_id: Optional[str] = None,
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Generate a response using RAG approach
        
        Args:
            query: User's question
            user_background: User's software/hardware background for personalization
            user_id: ID of the requesting user
            selected_text: Optional text provided by the user to restrict answers to
        
        Returns:
            Dictionary containing response and metadata
        """
        try:
            # 1. If selected text is provided, use only that as context
            context_texts = []
            if selected_text:
                context_texts = [selected_text]
                sources = []
            else:
                # 2. Generate embedding for the query
                query_embedding = await self.doc_processor.embedding_model.aembed_query(query)
                
                # 3. Search for relevant documents in vector database
                search_results = await self.vector_db.search_similar(
                    query_embedding, 
                    top_k=5
                )
                
                # Extract the content from search results
                context_texts = [result["content"] for result in search_results]
                sources = search_results

            # 4. Create personalized context based on user background
            context = self._create_context_with_user_background(
                query, context_texts, user_background
            )
            
            # 5. Generate response using LLM
            response = await self._generate_llm_response(context, query)
            
            # 6. Create response with metadata
            result = {
                "response": response,
                "sources": sources,
                "query": query,
                "model_used": "gpt-3.5-turbo"  # or whatever model you're using
            }
            
            # 7. Log the interaction
            await self._log_interaction(query, response, user_id, sources)
            
            return result
            
        except Exception as e:
            logging.error(f"Error in RAG generation: {e}")
            raise
    
    def _create_context_with_user_background(
        self, 
        query: str, 
        context_texts: List[str], 
        user_background: Optional[Dict[str, str]]
    ) -> str:
        """Create a context string with user background information for personalization"""
        # Start with background information if available
        background_context = ""
        if user_background:
            software_bg = user_background.get("software_background", "beginner")
            hardware_bg = user_background.get("hardware_background", "beginner")
            
            # Adjust the persona based on user's background
            if software_bg == "beginner" or hardware_bg == "beginner":
                background_context = "Explain concepts in simple terms with practical examples. "
            elif software_bg == "expert" or hardware_bg == "expert":
                background_context = "Provide detailed technical explanations with advanced concepts. "
            else:
                background_context = "Provide balanced explanations with appropriate detail. "
        
        # Combine background with the context from documents
        full_context = background_context + "\n\n"
        
        if context_texts:
            full_context += "Relevant information from the book:\n\n"
            for i, text in enumerate(context_texts):
                full_context += f"Source {i+1}:\n{text}\n\n"
        else:
            full_context += "No relevant information was found in the book. "
            
        return full_context
    
    async def _generate_llm_response(self, context: str, query: str) -> str:
        """Generate a response using the LLM with the provided context"""
        # Create the prompt for the LLM
        prompt = f"""
        You are a helpful assistant for a book. Answer the user's question based on the context provided below.
        
        Context: {context}
        
        User's question: {query}
        
        Instructions:
        - If the context contains relevant information, use it to answer the question.
        - If the context doesn't contain relevant information, state that you don't have enough information from the book to answer.
        - Be concise and helpful in your response.
        - Cite sources when possible using the source numbers mentioned in the context.
        """
        
        try:
            response = await self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",  # You can change this to gpt-4 if preferred
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that answers questions based on provided context from a book."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=1000,
                temperature=0.3,  # Lower temperature for more consistent responses
            )
            
            return response.choices[0].message.content
        
        except Exception as e:
            logging.error(f"Error generating LLM response: {e}")
            raise
    
    async def _log_interaction(
        self, 
        query: str, 
        response: str, 
        user_id: Optional[str], 
        sources: List[Dict]
    ):
        """Log the interaction for analytics and improvement"""
        # This would typically store in a database for analytics
        logging.info(f"Interaction logged - Query: {query[:50]}... | User ID: {user_id}")
```

## Chat API Endpoint

Now let's implement the API endpoint that connects to our RAG service:

```python
# backend/app/api/v1/chat.py
from fastapi import APIRouter, Depends, HTTPException, Request
from sqlalchemy.ext.asyncio import AsyncSession
from app.database import get_async_session
from app.models.user import UserRead
from app.api.deps import get_current_user
from app.services.rag_service import RAGService
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import logging

router = APIRouter()
rag_service = RAGService()

class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None  # Text selected by user to restrict answers to
    session_id: Optional[str] = None     # For multi-turn conversations

class ChatResponse(BaseModel):
    response: str
    sources: List[Dict]
    query: str
    model_used: str

@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    current_user: UserRead = Depends(get_current_user),
    session: AsyncSession = Depends(get_async_session)
):
    """
    Main chat endpoint for the RAG system
    """
    try:
        # Create user background context
        user_background = {
            "software_background": current_user.software_background,
            "hardware_background": current_user.hardware_background
        }
        
        # Generate response using RAG service
        result = await rag_service.generate_response(
            query=request.message,
            user_background=user_background,
            user_id=current_user.id,
            selected_text=request.selected_text
        )
        
        return ChatResponse(**result)
    
    except Exception as e:
        logging.error(f"Chat error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error during chat")

class DocumentSelectionRequest(BaseModel):
    query: str
    document_content: str  # The text content selected by the user

@router.post("/chat_with_selection")
async def chat_with_selection(
    request: DocumentSelectionRequest,
    current_user: UserRead = Depends(get_current_user),
    session: AsyncSession = Depends(get_async_session)
):
    """
    Chat endpoint that answers only based on the provided document content
    """
    try:
        # Create user background context
        user_background = {
            "software_background": current_user.software_background,
            "hardware_background": current_user.hardware_background
        }
        
        # Generate response using only the provided content
        result = await rag_service.generate_response(
            query=request.query,
            user_background=user_background,
            user_id=current_user.id,
            selected_text=request.document_content
        )
        
        return ChatResponse(**result)
    
    except Exception as e:
        logging.error(f"Chat with selection error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error during chat with selection")

class MultiTurnChatRequest(BaseModel):
    message: str
    session_id: Optional[str] = None
    selected_text: Optional[str] = None

@router.post("/chat_conversation")
async def chat_conversation(
    request: MultiTurnChatRequest,
    current_user: UserRead = Depends(get_current_user),
    session: AsyncSession = Depends(get_async_session)
):
    """
    Chat endpoint with conversation history management
    """
    try:
        # Create user background context
        user_background = {
            "software_background": current_user.software_background,
            "hardware_background": current_user.hardware_background
        }
        
        # For now, we'll implement a simple conversation without full history
        # In a real implementation, you'd retrieve conversation history from DB
        result = await rag_service.generate_response(
            query=request.message,
            user_background=user_background,
            user_id=current_user.id,
            selected_text=request.selected_text
        )
        
        # In a full implementation, you would:
        # 1. Generate or retrieve session_id
        # 2. Retrieve conversation history from DB
        # 3. Include history in the prompt to the LLM
        # 4. Save the interaction to DB for future context
        
        return ChatResponse(**result)
    
    except Exception as e:
        logging.error(f"Multi-turn chat error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error during conversation")
```

## API Dependencies and Security

Let's implement dependencies for user authentication and rate limiting:

```python
# backend/app/api/deps.py
from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.ext.asyncio import AsyncSession
from app.database import get_async_session
from app.models.user import User
from app.schemas.user import UserRead
from better_auth.client import Client
from better_auth.security import verify_token
import logging

security = HTTPBearer()

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db_session: AsyncSession = Depends(get_async_session)
) -> UserRead:
    """
    Get the current authenticated user from the token
    """
    try:
        token = credentials.credentials
        
        # Verify the token using Better Auth
        user_info = await verify_token(token)
        
        if not user_info:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid authentication credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        # Retrieve user from database
        user = await db_session.get(User, user_info.get("id"))
        if not user or not user.is_active:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User account is inactive",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        return UserRead.from_orm(user)
    
    except Exception as e:
        logging.error(f"Error getting current user: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

# Rate limiting implementation (simplified)
from functools import wraps
import time
from collections import defaultdict

# Simple in-memory rate limiter (use Redis in production)
user_requests = defaultdict(list)

def rate_limit(max_requests: int, window_seconds: int):
    """
    Rate limiting decorator
    """
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            user_id = kwargs.get('current_user').id if 'current_user' in kwargs else 'anonymous'
            
            now = time.time()
            
            # Clean old requests outside the window
            user_requests[user_id] = [
                req_time for req_time in user_requests[user_id] 
                if now - req_time < window_seconds
            ]
            
            # Check if user has exceeded the limit
            if len(user_requests[user_id]) >= max_requests:
                raise HTTPException(
                    status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                    detail="Rate limit exceeded"
                )
            
            # Add current request
            user_requests[user_id].append(now)
            
            return await func(*args, **kwargs)
        return wrapper
    return decorator
```

## Configuration Management

Let's complete the configuration file:

```python
# backend/app/config.py
from pydantic_settings import SettingsConfigDict, BaseSettings
from typing import List, Optional

class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=".env", case_sensitive=True)
    
    # API Settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "RAG Chatbot API"
    
    # Authentication
    AUTH_SECRET: str
    ALLOWED_ORIGINS: List[str] = ["http://localhost:3000", "http://localhost:8000"]
    
    # Database
    NEON_DATABASE_URL: str
    
    # Vector Database
    QDRANT_URL: str
    QDRANT_API_KEY: str
    
    # LLM Settings
    OPENAI_API_KEY: str
    GROK_API_KEY: Optional[str] = None  # Alternative LLM provider
    
    # Application
    DEBUG: bool = False

settings = Settings()
```

## Database Models Integration

Finally, let's make sure our user model is properly connected:

```python
# backend/app/models/user.py (updated)
from sqlmodel import SQLModel, Field
from datetime import datetime
from typing import Optional
from pydantic import BaseModel

class UserBase(BaseModel):
    email: str
    name: Optional[str] = None

class User(UserBase):
    id: Optional[int] = Field(default=None, primary_key=True)
    hashed_password: str
    software_background: str = "beginner"  # beginner, intermediate, advanced, expert
    hardware_background: str = "beginner"  # beginner, intermediate, advanced, expert
    is_active: bool = True
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

class UserCreate(UserBase):
    password: str
    software_background: str = "beginner"
    hardware_background: str = "beginner"

class UserRead(UserBase):
    id: int
    software_background: str
    hardware_background: str
    created_at: datetime
```

## Testing the Backend

To ensure everything works together, let's create a simple test:

```python
# backend/tests/test_rag_backend.py
import pytest
from fastapi.testclient import TestClient
from app.main import app
from unittest.mock import AsyncMock, patch

client = TestClient(app)

@pytest.mark.asyncio
async def test_chat_endpoint():
    """Test the chat endpoint"""
    # Mock the RAG service
    with patch('app.api.v1.chat.rag_service') as mock_rag_service:
        mock_rag_service.generate_response = AsyncMock(return_value={
            "response": "This is a test response",
            "sources": [],
            "query": "test query",
            "model_used": "gpt-3.5-turbo"
        })
        
        # Mock authentication
        with patch('app.api.deps.get_current_user') as mock_user:
            mock_user.return_value = type('User', (), {
                'id': 1,
                'software_background': 'beginner',
                'hardware_background': 'beginner',
                'is_active': True
            })()
            
            response = client.post(
                "/api/v1/chat",
                json={"message": "Hello, how does RAG work?"}
            )
            
            assert response.status_code == 200
            data = response.json()
            assert "response" in data
            assert "sources" in data
```

Continue to [Lesson 5: Creating the Professional Frontend UI](./lesson-5-frontend-ui.md) to build the user interface for our RAG chatbot.