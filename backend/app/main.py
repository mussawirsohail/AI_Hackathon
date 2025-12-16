from contextlib import asynccontextmanager
from fastapi import FastAPI
from app.api.v1.api import api_router
from app.config import settings
from app.services.rag_service import RAGService
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global variable to hold the RAG service instance
rag_service = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Initialize resources on startup
    logger.info("Starting up RAG Chatbot API")
    try:
        # Initialize RAG service
        rag_service_instance = RAGService()
        app.state.rag_service = rag_service_instance
        logger.info("RAG Service initialized successfully during startup")
    except Exception as e:
        logger.error(f"Failed to initialize RAG Service: {str(e)}")
        logger.error(f"Full traceback: {__import__('traceback').format_exc()}")
        raise
    yield
    # Clean up resources on shutdown
    logger.info("Shutting down RAG Chatbot API")

app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG API",
    description="API for Physical AI & Humanoid Robotics RAG chatbot",
    version="1.0.0",
    openapi_url="/api/v1/openapi.json",
    lifespan=lifespan
)

# Set up logging to show all logs including from dependencies
import sys
logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))
logging.getLogger().setLevel(logging.DEBUG)

# Add CORS middleware
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure this properly
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(api_router, prefix="/api/v1")

@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics RAG API is running!"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}