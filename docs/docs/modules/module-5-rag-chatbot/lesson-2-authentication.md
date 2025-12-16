---
sidebar_position: 2
---

# Lesson 2: Setting Up Authentication and User Profiles

## Introduction

In this lesson, we'll implement the authentication system for our RAG chatbot using Better Auth. User authentication is crucial for tracking user interactions, personalizing content based on user background, and maintaining secure access to the chatbot.

## Why Better Auth?

Better Auth is a modern authentication library that provides:

- Easy integration with modern frameworks
- Secure session management
- Support for multiple authentication methods
- TypeScript support
- Database flexibility
- Customizable user profiles

## Installation and Setup

First, let's set up Better Auth in our project. For the backend, we'll create the necessary configuration:

```javascript
// backend/auth/config.js
import { betterAuth } from "better-auth";
import { postgresAdapter } from "@better-auth/postgres-adapter";
import { neon } from "neon";

const client = neon(process.env.NEON_DATABASE_URL);

export const auth = betterAuth({
  database: postgresAdapter(client, {
    // Custom user schema to include background information
    user: {
      additionalFields: {
        softwareBackground: {
          type: "string",
          required: true
        },
        hardwareBackground: {
          type: "string",
          required: true
        }
      }
    }
  }),
  secret: process.env.AUTH_SECRET,
  emailAndPassword: {
    enabled: true
  }
});
```

## Creating the User Registration Flow

The registration flow includes a survey to collect user background information, which will be used for content personalization:

```javascript
// frontend/src/components/Auth/SignupForm.jsx
import React, { useState } from 'react';
import { useForm } from 'react-hook-form';
import { useAuth } from '@better-auth/react';

const SignupForm = () => {
  const { registerUser } = useAuth();
  const [background, setBackground] = useState({
    softwareBackground: '',
    hardwareBackground: ''
  });

  const { register, handleSubmit, formState: { errors } } = useForm();

  const onSubmit = async (data) => {
    try {
      // Combine user data with background information
      const userData = {
        ...data,
        ...background
      };
      
      await registerUser(userData);
    } catch (error) {
      console.error('Registration error:', error);
    }
  };

  return (
    <form onSubmit={handleSubmit(onSubmit)} className="max-w-md mx-auto p-6 bg-white rounded-lg shadow-md">
      <h2 className="text-2xl font-bold mb-6">Create Account</h2>
      
      <div className="mb-4">
        <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="email">
          Email
        </label>
        <input
          {...register('email', { required: 'Email is required' })}
          className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
          id="email"
          type="email"
        />
        {errors.email && <p className="text-red-500 text-xs italic">{errors.email.message}</p>}
      </div>
      
      <div className="mb-4">
        <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="password">
          Password
        </label>
        <input
          {...register('password', { 
            required: 'Password is required',
            minLength: {
              value: 8,
              message: 'Password must be at least 8 characters'
            }
          })}
          className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 mb-3 leading-tight focus:outline-none focus:shadow-outline"
          id="password"
          type="password"
        />
        {errors.password && <p className="text-red-500 text-xs italic">{errors.password.message}</p>}
      </div>
      
      <div className="mb-6">
        <label className="block text-gray-700 text-sm font-bold mb-2">
          Software Background
        </label>
        <select
          value={background.softwareBackground}
          onChange={(e) => setBackground({...background, softwareBackground: e.target.value})}
          className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
        >
          <option value="">Select your software background</option>
          <option value="beginner">Beginner</option>
          <option value="intermediate">Intermediate</option>
          <option value="advanced">Advanced</option>
          <option value="expert">Expert</option>
        </select>
      </div>
      
      <div className="mb-6">
        <label className="block text-gray-700 text-sm font-bold mb-2">
          Hardware Background
        </label>
        <select
          value={background.hardwareBackground}
          onChange={(e) => setBackground({...background, hardwareBackground: e.target.value})}
          className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
        >
          <option value="">Select your hardware background</option>
          <option value="beginner">Beginner</option>
          <option value="intermediate">Intermediate</option>
          <option value="advanced">Advanced</option>
          <option value="expert">Expert</option>
        </select>
      </div>
      
      <div className="flex items-center justify-between">
        <button
          className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded focus:outline-none focus:shadow-outline"
          type="submit"
        >
          Sign Up
        </button>
      </div>
    </form>
  );
};

export default SignupForm;
```

## Backend User Model with Background Information

We need to extend our user model to store the software and hardware background information:

```python
# backend/app/models/user.py
from sqlmodel import SQLModel, Field
from datetime import datetime
from typing import Optional

class UserBase(SQLModel):
    email: str = Field(unique=True, index=True)
    name: Optional[str] = Field(default=None)

class User(UserBase, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    hashed_password: str
    software_background: str = Field(default="beginner")  # beginner, intermediate, advanced, expert
    hardware_background: str = Field(default="beginner")  # beginner, intermediate, advanced, expert
    is_active: bool = Field(default=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    
class UserCreate(UserBase):
    password: str
    software_background: str = "beginner"
    hardware_background: str = "beginner"

class UserRead(UserBase):
    id: int
    software_background: str
    hardware_background: str
    created_at: datetime
```

## Database Migration

We'll also need to create a database migration to add the background fields to our user table:

```python
# backend/alembic/versions/xxx_add_background_fields_to_user.py
from alembic import op
import sqlalchemy as sa

# revision identifiers
revision = 'xxx'
down_revision = 'yyy'
branch_labels = None
depends_on = None

def upgrade():
    # Add software_background column
    op.add_column('user', sa.Column('software_background', sa.String(), nullable=True, server_default='beginner'))
    
    # Add hardware_background column
    op.add_column('user', sa.Column('hardware_background', sa.String(), nullable=True, server_default='beginner'))
    
    # Update existing records to have default values
    op.execute("UPDATE user SET software_background = 'beginner' WHERE software_background IS NULL")
    op.execute("UPDATE user SET hardware_background = 'beginner' WHERE hardware_background IS NULL")
    
    # Alter columns to be non-nullable
    op.alter_column('user', 'software_background', nullable=False)
    op.alter_column('user', 'hardware_background', nullable=False)

def downgrade():
    op.drop_column('user', 'hardware_background')
    op.drop_column('user', 'software_background')
```

## Personalization Based on User Background

Once we have user background information, we can personalize the chatbot responses:

```python
# backend/app/api/v1/chat.py
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from app.database import get_async_session
from app.models.user import UserRead
from app.api.deps import get_current_user
import logging

router = APIRouter()

@router.post("/chat")
async def chat_with_rag(
    query: str,
    session: AsyncSession = Depends(get_async_session),
    current_user: UserRead = Depends(get_current_user)
):
    """
    Chat endpoint that personalizes responses based on user background
    """
    try:
        # Personalize the query based on user background
        personalized_context = create_personalized_context(
            query, 
            current_user.software_background, 
            current_user.hardware_background
        )
        
        # Perform RAG operation with personalized context
        response = await perform_rag_query(personalized_context)
        
        return {"response": response}
    except Exception as e:
        logging.error(f"Chat error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")

def create_personalized_context(query: str, software_bg: str, hardware_bg: str):
    """
    Create personalized context based on user background
    """
    # Adjust the persona based on user's background
    if software_bg == "beginner" or hardware_bg == "beginner":
        persona = "Explain concepts in simple terms with practical examples."
    elif software_bg == "expert" or hardware_bg == "expert":
        persona = "Provide detailed technical explanations with advanced concepts."
    else:
        persona = "Provide balanced explanations with appropriate detail."
    
    return f"User Background: Software={software_bg}, Hardware={hardware_bg}. {persona} Query: {query}"
```

## Security Considerations

When implementing the authentication system, we must consider several security aspects:

1. **Password Security**: Ensure passwords are properly hashed using strong algorithms
2. **Session Security**: Implement secure session management with appropriate expiration
3. **Data Protection**: Encrypt sensitive user data in the database
4. **Rate Limiting**: Prevent abuse of authentication endpoints
5. **Input Validation**: Validate all user inputs to prevent injection attacks

Continue to [Lesson 3: Vector Database Setup with Qdrant](./lesson-3-vector-database.md) to learn how to set up the vector database for our RAG system.