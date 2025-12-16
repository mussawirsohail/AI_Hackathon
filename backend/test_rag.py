import asyncio
import os
from app.services.rag_service import RAGService
from app.api.v1.chat import ChatRequest

async def test_rag_service():
    """
    Test script to verify the RAG service is working correctly with Groq integration
    """
    print("Testing RAG service with Groq integration...")

    try:
        # Initialize the RAG service
        rag_service = RAGService()
        print("✓ RAG service initialized successfully")

        # Test a simple query
        test_query = "What is Physical AI?"
        print(f"Testing query: '{test_query}'")

        response = await rag_service.generate_response(test_query)
        print("✓ Response generated successfully")
        print(f"Response: {response['response'][:200]}...")

        # Test with context
        test_context = "Physical AI is a field that combines physics and artificial intelligence."
        context_response = await rag_service.generate_response_with_context(test_query, test_context)
        print("✓ Context-based response generated successfully")
        print(f"Context response: {context_response['response'][:200]}...")

        print("\n✓ All tests passed! The RAG pipeline is working correctly.")

    except Exception as e:
        print(f"✗ Error during testing: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    # Check if required environment variables are set
    required_vars = ['GROQ_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']
    missing_vars = [var for var in required_vars if not os.getenv(var)]
    
    if missing_vars:
        print(f"⚠️  Missing required environment variables: {missing_vars}")
        print("Please set them before running the test.")
    else:
        asyncio.run(test_rag_service())