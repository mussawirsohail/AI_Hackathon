# RAG (Retrieval-Augmented Generation) Chatbot

This RAG chatbot allows users to ask questions about the book content and receive AI-powered answers based on the documentation.

## Features

- **Context-aware responses**: Answers based on selected text
- **Floating chat interface**: Always accessible on any page
- **Persistent conversation history**: Maintains context during session
- **Source attribution**: Shows which documents contributed to the answer
- **Text selection integration**: Users can select text to provide context for questions

## Architecture

- **Frontend**: React component embedded in Docusaurus
- **Backend**: FastAPI server with OpenAI integration
- **Vector Store**: Qdrant for document embeddings
- **Database**: Neon Serverless Postgres for metadata
- **Language Model**: OpenAI GPT for response generation

## Setup Instructions

### 1. Install Backend Dependencies

```bash
cd rag_backend
pip install -r requirements.txt
```

**Note for Windows users:** If you encounter issues installing packages (especially numpy, psycopg2), you may need to install pre-compiled wheels:

```bash
pip install --only-binary=all numpy
pip install psycopg2-binary
pip install -r requirements.txt
```

For a simpler setup without RAG functionality, you can install just the authentication dependencies:

```bash
pip install fastapi uvicorn python-dotenv
```

### 2. Configure Environment Variables

Create a `.env` file in the `rag_backend` directory:

```bash
copy .env.example .env  # On Windows
# or cp .env.example .env on Linux/Mac
```

Then update the values:

- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_URL`: Your QDRant instance URL
- `QDRANT_API_KEY`: Your QDrant API key (if using cloud)
- `NEON_DB_URL`: Your Neon PostgreSQL connection string

### 3. Ingest Documentation

Run the ingestion script to populate the vector database:

```bash
python ingest_docs.py
```

### 4. Start the Backend Server

```bash
cd rag_backend
uvicorn main:app --reload
```

The backend will be available at `http://localhost:8000` with both RAG and authentication endpoints.

### 5. Run the Docusaurus Site

In a separate terminal:

```bash
cd docs
npm install
npm start
```

The chatbot will appear as a floating button on all documentation pages.

## How It Works

1. **Document Ingestion**: Documentation files are loaded, split into chunks, and stored as vector embeddings in Qdrant.
2. **Query Processing**: When a user submits a question, it's processed with any selected text as additional context.
3. **Retrieval**: Similar document chunks are retrieved from the vector store.
4. **Generation**: OpenAI generates a response based on the question and retrieved context.
5. **Response**: The answer is displayed in the chat interface along with source references.

## Customization

You can customize the chatbot by modifying:
- `docs/src/components/RAGChatbot.jsx`: Main chat interface
- `rag_backend/main.py`: Backend API logic
- `rag_backend/ingest_docs.py`: Document ingestion pipeline