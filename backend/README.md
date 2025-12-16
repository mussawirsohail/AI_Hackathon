# Physical AI & Humanoid Robotics RAG Chatbot

This project implements a RAG (Retrieval-Augmented Generation) chatbot for Physical AI & Humanoid Robotics documentation using Groq API instead of OpenAI.

## Requirements

- Python 3.8 or higher
- Groq API key (free tier available)
- Qdrant vector database instance
- Windows Long Path Support (recommended for Windows users, otherwise install packages separately)

## Setup

1. Install dependencies:

```bash
pip install groq sentence-transformers torch qdrant-client tenacity
```

2. Create a `.env` file in the backend directory with your credentials:

```env
# Qdrant Vector Database
QDRANT_URL=your-qdrant-cluster-url
QDRANT_API_KEY=your-qdrant-api-key

# Groq API
GROQ_API_KEY=your-groq-api-key
GROQ_MODEL_NAME=llama-3.1-8b-instant  # Or llama-3.1-70b-versatile

# Application
DEBUG=False
```

3. Make sure your Qdrant database contains the Physical AI & Humanoid Robotics documents in a collection named `physical_ai_content`.

## Running the Application

```bash
cd backend
uvicorn app.main:app --reload --port 8000
```

## API Usage

The chat API endpoint is available at:
- POST `/api/v1/chat/`

Example request body:
```json
{
  "message": "What is Physical AI?",
  "context_text": "Optional context provided by user"
}
```

## Features

- **RAG Pipeline**: Query vector database for relevant documents
- **Groq Integration**: Uses Groq's LLM (Llama 3 models) with free tier support
- **Detailed Logging**: Full logging at each step of the pipeline
- **Robust Error Handling**: Safe user-facing messages with full tracebacks in logs
- **Fallback Handling**: Graceful handling when no documents found
- **Retry & Timeout**: Automatic retries and proper timeout handling
- **Configurable Model**: Choose between llama3-8b-8192 or llama3-70b-8192

## Error Handling

- Full traceback logging in server logs
- Safe, user-friendly error messages
- Graceful fallback when no relevant documents found
- "I couldn't find this information in the book, but here's a general explanation..." response when no context available

## Models Supported

- `llama3-8b-8192` (default, suitable for most queries)
- `llama3-70b-8192` (more powerful, configured via GROQ_MODEL_NAME)

## Troubleshooting

1. If you encounter long path issues on Windows, install packages individually:
   ```bash
   pip install groq
   pip install sentence-transformers
   pip install torch
   ```

2. Make sure your Groq API key is valid and has sufficient quota

3. Ensure your Qdrant database is accessible and properly indexed

## Testing

Run the basic test to verify the RAG service:

```python
python -c "import asyncio; from test_rag import test_rag_service; asyncio.run(test_rag_service())"
```