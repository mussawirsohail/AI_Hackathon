from fastapi import APIRouter, HTTPException, Depends, Request
from pydantic import BaseModel
from typing import Optional, List
import asyncio
import logging
import traceback
from app.config import settings

router = APIRouter()

logger = logging.getLogger(__name__)

class ChatRequest(BaseModel):
    message: str
    context_text: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    sources: List[dict] = []

@router.post("/", response_model=ChatResponse)
async def chat(request: ChatRequest, req: Request):
    """
    Main chat endpoint that uses RAG to answer questions
    """
    try:
        # Get the RAG service instance from app state
        rag_service = req.app.state.rag_service

        logger.info(f"Received chat request with message: {request.message[:100]}...")

        # If context text is provided, use only that for answers
        if request.context_text:
            logger.info("Using provided context for response generation")
            response = await rag_service.generate_response_with_context(
                request.message,
                request.context_text
            )
        else:
            logger.info("Performing vector search for relevant documents")
            # Otherwise, search in the vector database for relevant content
            response = await rag_service.generate_response(
                request.message
            )

        logger.info("Successfully generated response")
        return ChatResponse(
            response=response["response"],
            sources=response.get("sources", [])
        )
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        logger.error(f"Full traceback: {traceback.format_exc()}")

        # For debugging, return more specific error information
        error_msg = str(e)
        if "context" in error_msg.lower() or "embedding" in error_msg.lower():
            user_msg = "Sorry, I'm having trouble accessing the knowledge base right now, but I'm still here to help. Could you try rephrasing your question?"
        elif "groq" in error_msg.lower() or "api" in error_msg.lower():
            user_msg = "Sorry, I'm having trouble connecting to the AI service right now. Please try again in a moment."
        elif "qdrant" in error_msg.lower() or "vector" in error_msg.lower():
            user_msg = "I couldn't find relevant information in the book, but here's a general explanation: Physical AI is an interdisciplinary field that combines physics principles with artificial intelligence techniques to create more robust and efficient AI systems."
        else:
            user_msg = "Sorry, I encountered an error processing your request. Please try again."

        # Return a safe, helpful user-facing message
        return ChatResponse(
            response=user_msg,
            sources=[]
        )