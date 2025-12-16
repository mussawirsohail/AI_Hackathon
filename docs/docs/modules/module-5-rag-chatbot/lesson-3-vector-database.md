---
sidebar_position: 3
---

# Lesson 3: Vector Database Setup with Qdrant

## Introduction to Vector Databases

Vector databases store and retrieve data based on similarity rather than exact matches. In a RAG system, the book content is converted into vector embeddings, which allow the system to find the most relevant text passages based on the user's query.

## Why Qdrant?

Qdrant is an open-source vector database that offers:

- High-performance similarity search
- Support for various distance metrics (Cosine, Euclidean, Dot Product)
- Filtering capabilities
- API-first design with SDKs in multiple languages
- Support for both local and cloud deployment
- Integration with popular embedding models

## Setting Up Qdrant Cloud Free Tier

1. Visit [qdrant.tech](https://qdrant.tech) and create an account
2. Create a new collection for your book content
3. Note your API key and cluster URL for later use

## Collection Schema Design

For our RAG chatbot, we'll create a collection schema that stores both the text content and metadata:

```python
# backend/app/vector_db.py
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
import uuid
from typing import List, Dict, Any
import logging

class VectorDBManager:
    def __init__(self, url: str, api_key: str):
        self.client = QdrantClient(
            url=url,
            api_key=api_key,
            prefer_grpc=True  # Use gRPC for better performance if available
        )
        self.collection_name = "book_content"
        self.vector_size = 1536  # Size for OpenAI embeddings; adjust for other models
        
    async def create_collection(self):
        """Create a collection in Qdrant for storing book content"""
        try:
            # Check if collection already exists
            collections = await self.client.get_collections()
            collection_names = [c.name for c in collections.collections]
            
            if self.collection_name not in collection_names:
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=self.vector_size,
                        distance=models.Distance.COSINE
                    )
                )
                
                # Create payload index for metadata
                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="document_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )
                
                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="page_number",
                    field_schema=models.PayloadSchemaType.INTEGER
                )
                
                logging.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logging.info(f"Collection already exists: {self.collection_name}")
                
        except Exception as e:
            logging.error(f"Error creating collection: {e}")
            raise

    async def add_documents(self, documents: List[Dict[str, Any]]):
        """Add documents to the Qdrant collection"""
        points = []
        
        for doc in documents:
            # Create a unique ID for each document segment
            point_id = str(uuid.uuid4())
            
            # Each document should have: id, content, embedding, metadata
            point = PointStruct(
                id=point_id,
                vector=doc['embedding'],
                payload={
                    "content": doc['content'],
                    "document_id": doc.get('document_id', ''),
                    "page_number": doc.get('page_number', 0),
                    "section_title": doc.get('section_title', ''),
                    "source_url": doc.get('source_url', ''),
                    "metadata": doc.get('metadata', {})
                }
            )
            
            points.append(point)
        
        # Upload in batches for efficiency
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i+batch_size]
            await self.client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
        
        logging.info(f"Uploaded {len(points)} documents to Qdrant")

    async def search_similar(self, query_embedding: List[float], top_k: int = 5, filters: Dict = None) -> List[Dict[str, Any]]:
        """Search for similar documents based on embedding"""
        try:
            # Build filter conditions if provided
            qdrant_filter = None
            if filters:
                conditions = []
                for key, value in filters.items():
                    conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )
                
                if conditions:
                    qdrant_filter = models.Filter(must=conditions)
            
            # Perform search
            search_results = await self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True,
                score_threshold=0.3,  # Filter out low-similarity matches
                query_filter=qdrant_filter
            )
            
            # Format results
            results = []
            for result in search_results:
                results.append({
                    "id": result.id,
                    "content": result.payload["content"],
                    "document_id": result.payload.get("document_id", ""),
                    "page_number": result.payload.get("page_number", 0),
                    "section_title": result.payload.get("section_title", ""),
                    "score": result.score  # Similarity score
                })
            
            return results
            
        except Exception as e:
            logging.error(f"Error searching Qdrant: {e}")
            raise
```

## Document Preprocessing Pipeline

Before storing documents in the vector database, we need to preprocess them:

```python
# backend/app/document_processor.py
import asyncio
from typing import List, Dict, Any
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain.embeddings.openai import OpenAIEmbeddings
import tiktoken
import logging

class DocumentProcessor:
    def __init__(self, embedding_model_name: str = "text-embedding-ada-002"):
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=200,
            length_function=self._get_token_count,
            separators=["\n\n", "\n", " ", ""]
        )
        self.embedding_model = OpenAIEmbeddings(model=embedding_model_name)
        
    def _get_token_count(self, text: str) -> int:
        """Get token count using tiktoken"""
        encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
        return len(encoding.encode(text))
    
    async def preprocess_document(self, content: str, metadata: Dict[str, Any] = None) -> List[Dict[str, Any]]:
        """Preprocess and split documents into chunks for embedding"""
        if metadata is None:
            metadata = {}
            
        # Split the document into chunks
        chunks = self.text_splitter.split_text(content)
        
        # Create documents with metadata
        documents = []
        for i, chunk in enumerate(chunks):
            doc = {
                "content": chunk,
                "document_id": metadata.get("document_id", f"doc_{hash(content[:50])}"),
                "page_number": metadata.get("page_number", i),
                "section_title": metadata.get("section_title", ""),
                "source_url": metadata.get("source_url", ""),
                "metadata": metadata
            }
            documents.append(doc)
        
        return documents
    
    async def generate_embeddings(self, documents: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Generate embeddings for documents in batches"""
        # Extract content for embedding
        contents = [doc["content"] for doc in documents]
        
        # Generate embeddings in batches to respect API limits
        batch_size = 100
        all_embeddings = []
        
        for i in range(0, len(contents), batch_size):
            batch = contents[i:i+batch_size]
            batch_embeddings = await self.embedding_model.aembed_documents(batch)
            
            # Combine documents with their embeddings
            for j, embedding in enumerate(batch_embeddings):
                doc_with_embedding = {**documents[i+j], "embedding": embedding}
                all_embeddings.append(doc_with_embedding)
        
        return all_embeddings
```

## Integration with FastAPI

We'll integrate the vector database functionality into our FastAPI application:

```python
# backend/app/api/v1/documents.py
from fastapi import APIRouter, Depends, HTTPException, UploadFile, File
from sqlalchemy.ext.asyncio import AsyncSession
from app.database import get_async_session
from app.models.user import UserRead
from app.api.deps import get_current_user
from app.vector_db import VectorDBManager
from app.document_processor import DocumentProcessor
from app.config import settings
import tempfile
import PyPDF2
import logging

router = APIRouter()

@router.post("/upload_document")
async def upload_document(
    file: UploadFile = File(...),
    session: AsyncSession = Depends(get_async_session),
    current_user: UserRead = Depends(get_current_user)
):
    """Upload and process a document for the RAG system"""
    try:
        # Validate file type
        if not file.content_type.startswith("text/") and file.content_type != "application/pdf":
            raise HTTPException(status_code=400, detail="Only text files and PDFs are supported")
        
        # Read file content
        content = await file.read()
        
        # Process based on file type
        if file.content_type == "application/pdf":
            content = extract_text_from_pdf(content)
        else:
            content = content.decode("utf-8")
        
        # Initialize vector DB and document processor
        vector_db = VectorDBManager(url=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY)
        doc_processor = DocumentProcessor()
        
        # Preprocess the document
        metadata = {
            "document_id": f"doc_{current_user.id}_{file.filename}",
            "source_url": f"/documents/{file.filename}",
            "uploaded_by": current_user.id
        }
        
        processed_docs = await doc_processor.preprocess_document(content, metadata)
        
        # Generate embeddings
        docs_with_embeddings = await doc_processor.generate_embeddings(processed_docs)
        
        # Add to vector database
        await vector_db.add_documents(docs_with_embeddings)
        
        return {
            "message": f"Successfully uploaded and processed {len(docs_with_embeddings)} document chunks",
            "chunks_count": len(docs_with_embeddings)
        }
        
    except Exception as e:
        logging.error(f"Document upload error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error during document processing")

def extract_text_from_pdf(pdf_content: bytes) -> str:
    """Extract text from PDF content"""
    with tempfile.NamedTemporaryFile(delete=False, suffix=".pdf") as temp_pdf:
        temp_pdf.write(pdf_content)
        temp_pdf.flush()
        
        with open(temp_pdf.name, 'rb') as pdf_file:
            reader = PyPDF2.PdfReader(pdf_file)
            text = ""
            for page in reader.pages:
                text += page.extract_text() + "\n"
    
    return text

@router.get("/search")
async def search_documents(
    query: str,
    top_k: int = 5,
    session: AsyncSession = Depends(get_async_session),
    current_user: UserRead = Depends(get_current_user)
):
    """Search for relevant documents based on the query"""
    try:
        # Initialize vector DB
        vector_db = VectorDBManager(url=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY)
        
        # Generate embedding for query
        doc_processor = DocumentProcessor()
        query_embedding = await doc_processor.embedding_model.aembed_query(query)
        
        # Search in vector database
        results = await vector_db.search_similar(query_embedding, top_k=top_k)
        
        return {"results": results}
        
    except Exception as e:
        logging.error(f"Document search error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error during search")
```

## Performance Optimization

To ensure optimal performance of our vector database:

1. **Proper Indexing**: Use HNSW (Hierarchical Navigable Small World) index for faster similarity search
2. **Batch Operations**: Upload documents in batches to reduce API overhead
3. **Score Thresholding**: Filter results to only include highly relevant matches
4. **Caching**: Cache frequently accessed embeddings to reduce API calls

## Configuring Qdrant Collection for Performance

```python
# backend/app/vector_db_config.py
from qdrant_client import QdrantClient
from qdrant_client.http import models
from app.config import settings

def configure_qdrant_collection():
    """Configure the Qdrant collection with optimized settings"""
    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY
    )
    
    # Create collection with optimized settings
    client.create_collection(
        collection_name="book_content",
        vectors_config=models.VectorParams(
            size=1536,  # Size for OpenAI embeddings
            distance=models.Distance.COSINE
        ),
        # Enable indexing for faster search
        hnsw_config=models.HnswConfigDiff(
            m=16,  # Size of every main index graph
            ef_construct=100,  # Number of neighbours to consider during construction
            full_scan_threshold=10000  # Use plain index if less elements
        ),
        # Enable quantization to reduce memory usage
        quantization_config=models.ScalarQuantization(
            type=models.QuantizationType.INT8,
            quantile=0.99
        )
    )
    
    # Create payload indexes for metadata filtering
    client.create_payload_index(
        collection_name="book_content",
        field_name="document_id",
        field_schema=models.PayloadSchemaType.KEYWORD
    )
    
    client.create_payload_index(
        collection_name="book_content",
        field_name="section_title",
        field_schema=models.PayloadSchemaType.TEXT
    )

if __name__ == "__main__":
    configure_qdrant_collection()
```

Continue to [Lesson 4: Building the RAG Backend with FastAPI](./lesson-4-rag-backend.md) to implement the complete RAG processing pipeline.