# RAG (Retrieval-Augmented Generation) Chatbot

<<<<<<< HEAD
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
=======
This website contains the comprehensive guide to Physical AI & Humanoid Robotics, a detailed educational resource covering the intersection of artificial intelligence and robotics, specifically focusing on creating systems that can interact with the physical world through humanoid forms.

## About This Book

This book is structured into four comprehensive modules, each containing four detailed lessons:

- **Module 1: The Robotic Nervous System (ROS 2)** - Middleware for robot control, covering ROS 2 fundamentals
- **Module 2: The Digital Twin (Gazebo & Unity)** - Physics simulation and environment building
- **Module 3: The AI-Robot Brain (NVIDIA Isaac™)** - Advanced perception and training
- **Module 4: Vision-Language-Action (VLA)** - The convergence of LLMs and Robotics

Each lesson emphasizes hands-on learning with practical examples and exercises that allow you to build functional prototypes.

## Target Audience

This book is designed for learners with a beginner to intermediate background who want to understand and implement humanoid robotics systems. We assume basic programming knowledge, ideally in Python or C++, and some familiarity with robotics concepts.

## Local Development

To run this book locally for development or review:

```bash
npm install
npm start
```
>>>>>>> 20a094f1c916911391067b9d1b2c0798214984d5

This command starts a local development server and opens up a browser window at `http://localhost:3000`. Most changes are reflected live without having to restart the server.

<<<<<<< HEAD
### 1. Install Backend Dependencies

```bash
cd rag_backend
pip install -r requirements.txt
```

**Note for Windows users:** If you encounter issues installing packages (especially numpy, psycopg2), you may need to install pre-compiled wheels:
=======
## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static content hosting service.
>>>>>>> 20a094f1c916911391067b9d1b2c0798214984d5

```bash
pip install --only-binary=all numpy
pip install psycopg2-binary
pip install -r requirements.txt
```

<<<<<<< HEAD
For a simpler setup without RAG functionality, you can install just the authentication dependencies:
=======
We welcome contributions to improve this educational resource. Please follow these steps:

1. Fork the repository
2. Create a feature branch for your changes
3. Add your changes with clear documentation
4. Submit a pull request with a clear description of your changes
>>>>>>> 20a094f1c916911391067b9d1b2c0798214984d5

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