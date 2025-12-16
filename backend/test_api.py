import asyncio
import os
import sys
from pathlib import Path

# Add the backend directory to the Python path
sys.path.append(str(Path(__file__).parent))

from app.api.v1.chat import chat, ChatRequest

async def test_api_endpoint():
    """
    Test script to verify the API endpoint is working correctly with Groq integration
    """
    print("Testing API endpoint with Groq integration...")
    
    try:
        # Create a test request
        test_request = ChatRequest(message="What is Physical AI?")
        
        # Call the API endpoint
        result = await chat(test_request)
        
        print("✓ API endpoint call successful")
        print(f"Response: {result.response[:200]}...")
        
        # Test with context
        test_request_with_context = ChatRequest(
            message="What is Physical AI?",
            context_text="Physical AI is a field that combines physics and artificial intelligence."
        )
        
        result_with_context = await chat(test_request_with_context)
        
        print("✓ API endpoint call with context successful")
        print(f"Context response: {result_with_context.response[:200]}...")
        
        print("\n✓ API tests passed! The API endpoint is working correctly.")
        
    except Exception as e:
        print(f"✗ Error during API testing: {str(e)}")
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
        asyncio.run(test_api_endpoint())