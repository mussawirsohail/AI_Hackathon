# RAG Chatbot Development Specification

## Overview
Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's content, including answering questions based only on text selected by the user.

## Features

### 1. RAG Chatbot Core Functionality
- Retrieve and process text from selected book content
- Answer questions based on the provided text context
- Handle multi-turn conversations
- Support for citations and source attribution

### 2. User Authentication System
- Sign up and sign in using Better Auth (https://www.better-auth.com/)
- User profile creation with software/hardware background survey
- Personalized content delivery based on user background
- Secure session management

### 3. Professional Frontend UI
- Modern, responsive design with professional CSS
- Intuitive chat interface
- User profile and settings management
- Content selection tools

### 4. Backend Services
- FastAPI-based RESTful API
- Neon Serverless Postgres for user data storage
- Qdrant Cloud Free Tier for vector storage and similarity search
- Proper data indexing and retrieval mechanisms

## Technical Stack
- Frontend: React.js, Tailwind CSS or Material-UI
- Backend: Python, FastAPI
- Authentication: Better Auth
- Vector Storage: Qdrant Cloud
- Database: Neon Serverless Postgres
- APIs: OpenAI API, Grok API
- Tools: Docker (optional)

## Requirements

### Functional Requirements
1. Users can authenticate using the signup/signin system
2. During signup, users must provide their software and hardware background
3. Users can select text from the book content
4. The RAG chatbot answers questions based on the selected text
5. The system personalizes content based on user background
6. The chatbot handles contextual conversations

### Non-functional Requirements
1. Professional, responsive UI design
2. Fast response times for query processing (< 3 seconds)
3. Secure authentication and data handling
4. Efficient vector search capabilities
5. Scalable architecture using serverless technologies
6. Proper error handling and user feedback

## Constraints
- Use Better Auth for authentication
- Utilize Qdrant Cloud Free Tier for vector storage
- Use Neon Serverless Postgres for user data
- Implement professional CSS (not basic designs)
- Use Grok API for RAG chatbot functionality