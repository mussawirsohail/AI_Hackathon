import asyncio
from typing import List, Dict, Any, Optional
from app.config import settings
from qdrant_client import QdrantClient
import logging
from groq import Groq
import time
import traceback

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        try:
            # Initialize Qdrant client
            self.qdrant_client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
            )
            logger.info("Qdrant client initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {str(e)}")
            raise

        # Initialize Groq client
        self.groq_client = Groq(api_key=settings.GROQ_API_KEY)
        logger.info("Groq client initialized successfully")

        # Instead of sentence transformers (which causes DLL issues on Windows),
        # we'll implement a basic keyword-based matching approach for now
        # In a real implementation, you might use a service like OpenAI's embedding API
        # or a different lightweight embedding solution
        self.collection_name = "physical_ai_content"

    def embed_text(self, text: str) -> List[float]:
        """
        Simple text embedding function (placeholder for when sentence transformers isn't available)
        This is a basic implementation - in production, use proper embedding service
        """
        # For now, we'll just return a dummy embedding that will be used for simple matching
        # This is very basic and should be replaced with a proper solution
        import hashlib
        hash_object = hashlib.sha256(text.encode())
        hex_dig = hash_object.hexdigest()
        # Convert hex digest to a list of floats (simplified)
        embedding = []
        for i in range(0, len(hex_dig), 2):
            if i + 1 < len(hex_dig):
                embedding.append(int(hex_dig[i:i+2], 16) / 255.0)
            else:
                embedding.append(int(hex_dig[i], 16) / 15.0)
        # Pad or truncate to a fixed size (e.g., 32 dimensions)
        while len(embedding) < 32:
            embedding.append(0.0)
        return embedding[:32]

    async def generate_response(self, query: str) -> Dict[str, Any]:
        """
        Generate a response using RAG approach
        """
        try:
            logger.info(f"Processing query: {query[:100]}...")

            # Generate embedding for the query
            logger.info("Generating embedding for query...")
            query_embedding = self.embed_text(query)
            logger.info(f"Query embedding generated with dimension: {len(query_embedding)}")

            # Search for relevant documents in vector database
            logger.info("Searching for relevant documents in vector database...")
            try:
                search_results = self.qdrant_client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    limit=5,
                    score_threshold=0.3  # Filter out low-similarity matches
                )
                logger.info(f"Found {len(search_results)} relevant documents")
            except Exception as search_error:
                logger.error(f"Error during vector database search: {str(search_error)}")
                logger.error(f"Full traceback: {traceback.format_exc()}")
                # If search fails, return empty results instead of crashing
                search_results = []
                logger.info("Proceeding with empty search results")

            # Extract the content from search results
            context_texts = [result.payload["content"] for result in search_results if "content" in result.payload]
            sources = [
                {
                    "id": result.id,
                    "content": result.payload.get("content", "")[:200] + "...",  # Truncate for performance
                    "score": result.score,
                    "metadata": result.payload.get("metadata", {})
                }
                for result in search_results
                if "content" in result.payload
            ]

            logger.info(f"Retrieved {len(context_texts)} valid document(s)")

            # Create context from retrieved documents
            if context_texts:
                logger.info("Building context from retrieved documents...")
                context = "Relevant information from Physical AI & Humanoid Robotics book:\n\n"
                for i, text in enumerate(context_texts):
                    context += f"Document {i+1}: {text}\n\n"
                logger.info(f"Context length: {len(context)} characters")
            else:
                logger.warning("No relevant documents found in vector search")
                context = "No relevant information was found in the Physical AI & Humanoid Robotics book. "

            # Generate response using the LLM
            prompt = f"""
            You are an assistant for the Physical AI & Humanoid Robotics book.
            Answer the user's question based on the context provided below.

            Context: {context}

            User's question: {query}

            Instructions:
            - If the context contains relevant information, use it to answer the question.
            - If the context doesn't contain relevant information, say "I couldn't find this information in the book, but here's a general explanation..." and then provide a helpful response.
            - Be concise and helpful in your response.
            - Focus specifically on Physical AI and Humanoid Robotics topics.
            - If no context is available, provide a relevant general answer based on your knowledge of physical AI and robotics.
            """

            logger.info("Sending request to Groq API...")

            # Using Groq model specified in settings
            response = self.groq_client.chat.completions.create(
                model=settings.GROQ_MODEL_NAME,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant for a Physical AI & Humanoid Robotics book. Answer questions based on the provided context."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=1000,
                temperature=0.3,
                timeout=30  # 30 seconds timeout
            )

            logger.info("Received response from Groq API")

            return {
                "response": response.choices[0].message.content,
                "sources": sources
            }

        except Exception as e:
            logger.error(f"Error in generate_response: {str(e)}")
            logger.error(f"Full traceback: {traceback.format_exc()}")
            raise

    async def generate_response_with_context(self, query: str, context_text: str) -> Dict[str, Any]:
        """
        Generate a response using only the provided context text
        """
        try:
            logger.info(f"Generating response with provided context for query: {query[:100]}...")

            # Create context from provided text
            context = f"Content provided by user:\n\n{context_text}\n\n"

            # Generate response using the LLM
            prompt = f"""
            You are an assistant for the Physical AI & Humanoid Robotics book.
            Answer the user's question based ONLY on the content provided below.

            Content: {context}

            User's question: {query}

            Instructions:
            - Answer the question based ONLY on the provided content.
            - If the content doesn't contain relevant information to answer the question, say "I couldn't find this information in the book, but here's a general explanation..." and then provide a helpful response.
            - Be concise and helpful in your response.
            - Focus specifically on Physical AI and Humanoid Robotics topics.
            """

            logger.info("Sending request to Groq API with provided context...")

            response = self.groq_client.chat.completions.create(
                model=settings.GROQ_MODEL_NAME,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant for a Physical AI & Humanoid Robotics book. Answer questions based only on the provided content."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=1000,
                temperature=0.3,
                timeout=30  # 30 seconds timeout
            )

            logger.info("Received response from Groq API with provided context")

            return {
                "response": response.choices[0].message.content,
                "sources": [{"id": "user_provided", "content": context_text[:200] + "...", "score": 1.0}]
            }

        except Exception as e:
            logger.error(f"Error in generate_response_with_context: {str(e)}")
            logger.error(f"Full traceback: {traceback.format_exc()}")
            raise