---
sidebar_position: 5
---

# Lesson 5: Creating the Professional Frontend UI

## Introduction

In this lesson, we'll build a professional, responsive frontend UI for our RAG chatbot using React and a modern CSS framework. The UI will include authentication flows, a chat interface, document selection tools, and user profile management.

## Setting Up the React Project

First, let's set up our React project with TypeScript and a professional CSS framework:

```bash
# Create React app with TypeScript
npx create-react-app rag-chatbot-frontend --template typescript

# Navigate to project directory
cd rag-chatbot-frontend

# Install required dependencies
npm install axios react-router-dom @headlessui/react @heroicons/react
npm install better-auth better-auth-react
npm install @tanstack/react-query  # For state management
npm install tailwindcss postcss autoprefixer  # For styling
npm install @types/node @types/react @types/react-dom  # TypeScript types
```

## Tailwind CSS Configuration

```javascript
// tailwind.config.js
/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        primary: {
          50: '#eff6ff',
          500: '#3b82f6',
          600: '#2563eb',
          700: '#1d4ed8',
        },
        secondary: {
          500: '#8b5cf6',
          600: '#7c3aed',
        }
      }
    },
  },
  plugins: [],
}
```

## Main App Component

```tsx
// src/App.tsx
import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import { QueryClient, QueryClientProvider } from '@tanstack/react-query';
import { AuthProvider, useAuth } from 'better-auth-react';
import { auth } from './auth/config';
import { Home } from './pages/Home';
import { Chat } from './pages/Chat';
import { Profile } from './pages/Profile';
import { Auth } from './pages/Auth';
import { Navbar } from './components/Navbar';
import { Footer } from './components/Footer';
import './App.css';

// Create a client
const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      retry: 1,
      staleTime: 1000 * 60 * 5, // 5 minutes
    },
  },
});

function AppContent() {
  const { isSignedIn } = useAuth();

  return (
    <div className="flex flex-col min-h-screen bg-gray-50">
      <Navbar />
      <main className="flex-grow">
        <Routes>
          <Route path="/" element={<Home />} />
          <Route 
            path="/chat" 
            element={isSignedIn ? <Chat /> : <Auth />} 
          />
          <Route 
            path="/profile" 
            element={isSignedIn ? <Profile /> : <Auth />} 
          />
          <Route path="/auth" element={<Auth />} />
        </Routes>
      </main>
      <Footer />
    </div>
  );
}

function App() {
  return (
    <QueryClientProvider client={queryClient}>
      <AuthProvider auth={auth}>
        <Router>
          <AppContent />
        </Router>
      </AuthProvider>
    </QueryClientProvider>
  );
}

export default App;
```

## Authentication Pages and Components

```tsx
// src/pages/Auth.tsx
import React, { useState } from 'react';
import { useAuth } from 'better-auth-react';
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from '../components/ui/Card';
import { Button } from '../components/ui/Button';
import { Input } from '../components/ui/Input';
import { Label } from '../components/ui/Label';
import { Link } from 'react-router-dom';

export const Auth = () => {
  const [isLogin, setIsLogin] = useState(true);
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    softwareBackground: 'beginner',
    hardwareBackground: 'beginner'
  });
  
  const { signIn, signUp } = useAuth();

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    try {
      if (isLogin) {
        await signIn('email-password', {
          email: formData.email,
          password: formData.password
        });
      } else {
        await signUp('email-password', {
          email: formData.email,
          password: formData.password,
          name: formData.name,
          software_background: formData.softwareBackground,
          hardware_background: formData.hardwareBackground
        });
      }
    } catch (error) {
      console.error('Authentication error:', error);
      alert('Authentication failed. Please try again.');
    }
  };

  return (
    <div className="min-h-screen flex items-center justify-center bg-gradient-to-br from-primary-50 to-secondary-50 p-4">
      <Card className="w-full max-w-md">
        <CardHeader className="text-center">
          <CardTitle className="text-2xl font-bold">
            {isLogin ? 'Sign In' : 'Create Account'}
          </CardTitle>
          <CardDescription>
            {isLogin 
              ? 'Sign in to access the RAG chatbot' 
              : 'Create an account to get started with the RAG chatbot'}
          </CardDescription>
        </CardHeader>
        <form onSubmit={handleSubmit}>
          <CardContent className="space-y-4">
            {!isLogin && (
              <div>
                <Label htmlFor="name">Full Name</Label>
                <Input
                  id="name"
                  name="name"
                  type="text"
                  value={formData.name}
                  onChange={handleChange}
                  required={!isLogin}
                />
              </div>
            )}
            
            <div>
              <Label htmlFor="email">Email</Label>
              <Input
                id="email"
                name="email"
                type="email"
                value={formData.email}
                onChange={handleChange}
                required
              />
            </div>
            
            <div>
              <Label htmlFor="password">Password</Label>
              <Input
                id="password"
                name="password"
                type="password"
                value={formData.password}
                onChange={handleChange}
                required
              />
            </div>
            
            {!isLogin && (
              <>
                <div>
                  <Label htmlFor="softwareBackground">Software Background</Label>
                  <select
                    id="softwareBackground"
                    name="softwareBackground"
                    value={formData.softwareBackground}
                    onChange={handleChange}
                    className="w-full p-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-primary-500"
                  >
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                    <option value="expert">Expert</option>
                  </select>
                </div>
                
                <div>
                  <Label htmlFor="hardwareBackground">Hardware Background</Label>
                  <select
                    id="hardwareBackground"
                    name="hardwareBackground"
                    value={formData.hardwareBackground}
                    onChange={handleChange}
                    className="w-full p-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-primary-500"
                  >
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                    <option value="expert">Expert</option>
                  </select>
                </div>
              </>
            )}
          </CardContent>
          
          <CardFooter className="flex flex-col">
            <Button type="submit" className="w-full">
              {isLogin ? 'Sign In' : 'Sign Up'}
            </Button>
            
            <div className="mt-4 text-center text-sm">
              {isLogin ? "Don't have an account?" : "Already have an account?"}
              <button
                type="button"
                className="ml-1 text-primary-600 hover:underline"
                onClick={() => setIsLogin(!isLogin)}
              >
                {isLogin ? 'Sign up' : 'Sign in'}
              </button>
            </div>
          </CardFooter>
        </form>
      </Card>
    </div>
  );
};
```

## Chat Interface Component

```tsx
// src/pages/Chat.tsx
import React, { useState, useRef, useEffect } from 'react';
import { useQuery, useMutation, useQueryClient } from '@tanstack/react-query';
import { useAuth } from 'better-auth-react';
import { Button } from '../components/ui/Button';
import { Input } from '../components/ui/Input';
import { Textarea } from '../components/ui/Textarea';
import { Select } from '../components/ui/Select';
import { MessageBubble } from '../components/MessageBubble';
import { DocumentSelector } from '../components/DocumentSelector';
import { ChatHistory } from '../components/ChatHistory';
import { apiClient } from '../services/api';

export const Chat = () => {
  const [inputMessage, setInputMessage] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [isProcessing, setIsProcessing] = useState(false);
  const [messages, setMessages] = useState<Array<{
    id: string;
    content: string;
    role: 'user' | 'assistant';
    timestamp: Date;
    sources?: Array<{id: string, content: string}>;
  }>>([]);
  
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const textareaRef = useRef<HTMLTextAreaElement>(null);
  const { user } = useAuth();
  const queryClient = useQueryClient();

  // Fetch chat history
  const { data: chatHistory, isLoading: isLoadingHistory } = useQuery({
    queryKey: ['chatHistory', user?.id],
    queryFn: async () => {
      // In a real implementation, fetch chat history from backend
      return [];
    },
    enabled: !!user?.id
  });

  // Mutation for sending messages
  const sendMessageMutation = useMutation({
    mutationFn: async (message: string) => {
      const response = await apiClient.post('/chat', {
        message,
        selected_text: selectedText
      });
      return response.data;
    },
    onMutate: async (newMessage) => {
      // Optimistically update UI
      const userMessage = {
        id: Date.now().toString(),
        content: newMessage,
        role: 'user' as const,
        timestamp: new Date()
      };
      
      setMessages(prev => [...prev, userMessage]);
      setInputMessage('');
      setIsProcessing(true);
    },
    onSuccess: (data) => {
      const botMessage = {
        id: `resp_${Date.now()}`,
        content: data.response,
        role: 'assistant' as const,
        timestamp: new Date(),
        sources: data.sources
      };
      
      setMessages(prev => [...prev, botMessage]);
      setSelectedText(''); // Clear selected text after sending
    },
    onError: (error) => {
      console.error('Error sending message:', error);
      setMessages(prev => [...prev, {
        id: `error_${Date.now()}`,
        content: 'Sorry, I encountered an error processing your request.',
        role: 'assistant' as const,
        timestamp: new Date()
      }]);
    },
    onSettled: () => {
      setIsProcessing(false);
    }
  });

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (inputMessage.trim() && !isProcessing) {
      sendMessageMutation.mutate(inputMessage);
    }
  };

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Auto-resize textarea
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = `${Math.min(textareaRef.current.scrollHeight, 200)}px`;
    }
  }, [inputMessage]);

  return (
    <div className="h-screen flex flex-col bg-white">
      {/* Header */}
      <header className="bg-primary-600 text-white p-4 shadow-md">
        <div className="container mx-auto flex justify-between items-center">
          <h1 className="text-xl font-bold">Book RAG Chatbot</h1>
          <div className="flex items-center space-x-4">
            <span className="text-sm">Welcome, {user?.name || user?.email}</span>
            <DocumentSelector onTextSelect={setSelectedText} />
          </div>
        </div>
      </header>

      <div className="flex flex-1 overflow-hidden">
        {/* Sidebar for chat history */}
        <div className="w-64 bg-gray-100 border-r border-gray-200 hidden md:block">
          <ChatHistory history={chatHistory || []} isLoading={isLoadingHistory} />
        </div>

        {/* Main chat area */}
        <div className="flex-1 flex flex-col">
          {/* Messages container */}
          <div className="flex-1 overflow-y-auto p-4 bg-gray-50">
            <div className="max-w-4xl mx-auto">
              {messages.length === 0 ? (
                <div className="text-center py-12">
                  <h2 className="text-2xl font-semibold text-gray-700 mb-4">Welcome to the Book RAG Chatbot</h2>
                  <p className="text-gray-600 mb-6">
                    Ask me questions about the book content, or select text to get answers based on specific passages.
                  </p>
                  
                  {/* Example questions */}
                  <div className="grid grid-cols-1 md:grid-cols-2 gap-3 max-w-2xl mx-auto">
                    {[
                      "What are the key concepts in chapter 1?",
                      "Explain the neural network architecture described in the book",
                      "How does quantum computing relate to AI?",
                      "What are the ethical considerations mentioned?"
                    ].map((question, index) => (
                      <button
                        key={index}
                        onClick={() => setInputMessage(question)}
                        className="bg-white border border-gray-200 rounded-lg p-3 text-left text-sm hover:bg-gray-50 transition-colors"
                      >
                        {question}
                      </button>
                    ))}
                  </div>
                </div>
              ) : (
                messages.map((message) => (
                  <MessageBubble 
                    key={message.id} 
                    message={message} 
                  />
                ))
              )}
              <div ref={messagesEndRef} />
            </div>
          </div>

          {/* Selected text indicator */}
          {selectedText && (
            <div className="bg-blue-50 border-t border-b border-blue-200 p-3">
              <div className="flex items-start">
                <span className="font-medium text-blue-700 mr-2">Selected Text:</span>
                <p className="text-sm text-gray-700 flex-1 line-clamp-2">{selectedText.substring(0, 200)}{selectedText.length > 200 ? '...' : ''}</p>
              </div>
            </div>
          )}

          {/* Input area */}
          <div className="bg-white border-t border-gray-200 p-4">
            <form onSubmit={handleSubmit} className="max-w-4xl mx-auto">
              <div className="flex flex-col space-y-3">
                <Textarea
                  ref={textareaRef}
                  value={inputMessage}
                  onChange={(e) => setInputMessage(e.target.value)}
                  placeholder="Ask a question about the book content..."
                  disabled={isProcessing}
                  className="min-h-[60px] max-h-48 resize-none"
                />
                <div className="flex justify-between items-center">
                  <div className="text-xs text-gray-500">
                    {selectedText ? "Response will be based on selected text" : "Ask anything about the book"}
                  </div>
                  <Button 
                    type="submit" 
                    disabled={isProcessing || !inputMessage.trim()}
                    className="px-6"
                  >
                    {isProcessing ? (
                      <span className="flex items-center">
                        <span className="animate-spin rounded-full h-4 w-4 border-b-2 border-white mr-2"></span>
                        Thinking...
                      </span>
                    ) : (
                      'Send'
                    )}
                  </Button>
                </div>
              </div>
            </form>
          </div>
        </div>
      </div>
    </div>
  );
};
```

## Message Bubble Component

```tsx
// src/components/MessageBubble.tsx
import React from 'react';
import { format } from 'date-fns';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
  sources?: Array<{id: string, content: string}>;
}

interface MessageBubbleProps {
  message: Message;
}

export const MessageBubble: React.FC<MessageBubbleProps> = ({ message }) => {
  const isUser = message.role === 'user';
  
  return (
    <div className={`flex mb-4 ${isUser ? 'justify-end' : 'justify-start'}`}>
      <div 
        className={`max-w-[80%] rounded-2xl px-4 py-3 ${
          isUser 
            ? 'bg-primary-600 text-white rounded-tr-none' 
            : 'bg-gray-200 text-gray-800 rounded-tl-none'
        }`}
      >
        <div className="whitespace-pre-wrap">{message.content}</div>
        
        {/* Timestamp */}
        <div className={`text-xs mt-1 ${isUser ? 'text-primary-200' : 'text-gray-500'}`}>
          {format(new Date(message.timestamp), 'h:mm a')}
        </div>
        
        {/* Sources for assistant messages */}
        {!isUser && message.sources && message.sources.length > 0 && (
          <div className="mt-3 pt-3 border-t border-gray-300 border-opacity-50">
            <div className="text-xs font-medium mb-1">Sources:</div>
            {message.sources.map((source, index) => (
              <div 
                key={source.id || index} 
                className="text-xs bg-white bg-opacity-20 rounded p-2 mt-1 truncate"
              >
                {source.content.substring(0, 100)}...
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
};
```

## Document Selection Component

```tsx
// src/components/DocumentSelector.tsx
import React, { useState } from 'react';
import { Button } from './ui/Button';
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogTrigger } from './ui/Dialog';
import { Textarea } from './ui/Textarea';

interface DocumentSelectorProps {
  onTextSelect: (text: string) => void;
}

export const DocumentSelector: React.FC<DocumentSelectorProps> = ({ onTextSelect }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedText, setSelectedText] = useState('');

  const handleSubmit = () => {
    if (selectedText.trim()) {
      onTextSelect(selectedText);
      setIsOpen(false);
      setSelectedText('');
    }
  };

  const handlePaste = async () => {
    try {
      const text = await navigator.clipboard.readText();
      setSelectedText(text);
    } catch (err) {
      console.error('Failed to read clipboard contents: ', err);
    }
  };

  return (
    <Dialog open={isOpen} onOpenChange={setIsOpen}>
      <DialogTrigger asChild>
        <Button variant="outline" className="text-sm">
          Select Text
        </Button>
      </DialogTrigger>
      <DialogContent className="sm:max-w-md">
        <DialogHeader>
          <DialogTitle>Select Document Text</DialogTitle>
        </DialogHeader>
        <div className="space-y-4 py-4">
          <p className="text-sm text-gray-600">
            Provide text content to restrict the chatbot's answers to this specific content only.
          </p>
          <Textarea
            value={selectedText}
            onChange={(e) => setSelectedText(e.target.value)}
            placeholder="Paste or type the text you want the chatbot to answer from..."
            rows={6}
          />
          <div className="flex space-x-2">
            <Button type="button" onClick={handlePaste} variant="outline" size="sm">
              Paste from Clipboard
            </Button>
            <Button 
              type="button" 
              onClick={handleSubmit} 
              disabled={!selectedText.trim()}
              size="sm"
            >
              Use Text
            </Button>
          </div>
        </div>
      </DialogContent>
    </Dialog>
  );
};
```

## User Profile Component

```tsx
// src/pages/Profile.tsx
import React, { useState, useEffect } from 'react';
import { useQuery, useMutation, useQueryClient } from '@tanstack/react-query';
import { useAuth } from 'better-auth-react';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../components/ui/Card';
import { Button } from '../components/ui/Button';
import { Input } from '../components/ui/Input';
import { Select } from '../components/ui/Select';
import { apiClient } from '../services/api';

export const Profile = () => {
  const { user, updateUser } = useAuth();
  const queryClient = useQueryClient();
  const [formState, setFormState] = useState({
    name: user?.name || '',
    email: user?.email || '',
    software_background: user?.software_background || 'beginner',
    hardware_background: user?.hardware_background || 'beginner'
  });

  const updateProfileMutation = useMutation({
    mutationFn: async (updatedData: typeof formState) => {
      const response = await apiClient.put('/users/profile', updatedData);
      return response.data;
    },
    onSuccess: (data) => {
      // Update the auth context
      if (user) {
        updateUser({
          ...user,
          ...data
        });
      }
      // Invalidate queries to refresh data
      queryClient.invalidateQueries({ queryKey: ['user', user?.id] });
    },
    onError: (error) => {
      console.error('Error updating profile:', error);
      alert('Failed to update profile. Please try again.');
    }
  });

  useEffect(() => {
    if (user) {
      setFormState({
        name: user.name || '',
        email: user.email || '',
        software_background: user.software_background || 'beginner',
        hardware_background: user.hardware_background || 'beginner'
      });
    }
  }, [user]);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    updateProfileMutation.mutate(formState);
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormState(prev => ({ ...prev, [name]: value }));
  };

  if (!user) {
    return <div>Loading...</div>;
  }

  return (
    <div className="container mx-auto p-6 max-w-3xl">
      <Card>
        <CardHeader>
          <CardTitle>User Profile</CardTitle>
          <CardDescription>Manage your account and personalization settings</CardDescription>
        </CardHeader>
        <form onSubmit={handleSubmit}>
          <CardContent className="space-y-6">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              <div>
                <label htmlFor="name" className="block text-sm font-medium text-gray-700 mb-1">
                  Full Name
                </label>
                <Input
                  id="name"
                  name="name"
                  value={formState.name}
                  onChange={handleChange}
                  required
                />
              </div>
              
              <div>
                <label htmlFor="email" className="block text-sm font-medium text-gray-700 mb-1">
                  Email
                </label>
                <Input
                  id="email"
                  name="email"
                  type="email"
                  value={formState.email}
                  onChange={handleChange}
                  disabled
                  className="bg-gray-100"
                />
              </div>
            </div>
            
            <div>
              <label htmlFor="software_background" className="block text-sm font-medium text-gray-700 mb-1">
                Software Background
              </label>
              <Select
                id="software_background"
                name="software_background"
                value={formState.software_background}
                onChange={handleChange}
              >
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
                <option value="expert">Expert</option>
              </Select>
            </div>
            
            <div>
              <label htmlFor="hardware_background" className="block text-sm font-medium text-gray-700 mb-1">
                Hardware Background
              </label>
              <Select
                id="hardware_background"
                name="hardware_background"
                value={formState.hardware_background}
                onChange={handleChange}
              >
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
                <option value="expert">Expert</option>
              </Select>
            </div>
            
            <div className="pt-4">
              <Button 
                type="submit" 
                disabled={updateProfileMutation.isPending}
                className="w-full md:w-auto"
              >
                {updateProfileMutation.isPending ? 'Updating...' : 'Update Profile'}
              </Button>
            </div>
          </CardContent>
        </form>
      </Card>
      
      {/* Personalization Preview */}
      <Card className="mt-6">
        <CardHeader>
          <CardTitle>Personalization Settings</CardTitle>
          <CardDescription>How we use your background to tailor responses</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="bg-blue-50 border border-blue-200 rounded-lg p-4">
            <h3 className="font-medium text-blue-800 mb-2">Your Background:</h3>
            <ul className="list-disc pl-5 space-y-1">
              <li>Software: <span className="capitalize">{formState.software_background}</span></li>
              <li>Hardware: <span className="capitalize">{formState.hardware_background}</span></li>
            </ul>
            <p className="mt-3 text-sm text-gray-700">
              Based on your background, the chatbot will adjust its responses to match your 
              experience level, providing more detailed explanations for beginners or 
              concise technical information for experts.
            </p>
          </div>
        </CardContent>
      </Card>
    </div>
  );
};
```

## API Service

```tsx
// src/services/api.ts
import axios from 'axios';

const BASE_URL = process.env.REACT_APP_API_URL || 'https://mussawirsoomro5-physical-ai.hf.space/api/v1';

export const apiClient = axios.create({
  baseURL: BASE_URL,
  timeout: 30000,
  headers: {
    'Content-Type': 'application/json',
  },
});

// Request interceptor to add auth token
apiClient.interceptors.request.use(
  (config) => {
    const token = localStorage.getItem('auth_token');
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  (error) => {
    return Promise.reject(error);
  }
);

// Response interceptor to handle auth errors
apiClient.interceptors.response.use(
  (response) => {
    return response;
  },
  (error) => {
    if (error.response?.status === 401) {
      // Clear auth token and redirect to login
      localStorage.removeItem('auth_token');
      window.location.href = '/auth';
    }
    return Promise.reject(error);
  }
);
```

## UI Components

```tsx
// src/components/ui/Button.tsx
import React from 'react';

interface ButtonProps extends React.ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: 'default' | 'outline' | 'ghost' | 'link';
  size?: 'sm' | 'md' | 'lg';
}

export const Button: React.FC<ButtonProps> = ({ 
  children, 
  variant = 'default', 
  size = 'md', 
  className = '', 
  ...props 
}) => {
  const baseClasses = "inline-flex items-center justify-center rounded-md font-medium transition-colors focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring disabled:pointer-events-none disabled:opacity-50";
  
  const variantClasses = {
    default: "bg-primary-600 text-white hover:bg-primary-700",
    outline: "border border-input bg-transparent hover:bg-accent hover:text-accent-foreground",
    ghost: "hover:bg-accent hover:text-accent-foreground",
    link: "text-primary-600 underline-offset-4 hover:underline"
  };
  
  const sizeClasses = {
    sm: "h-9 px-3 text-sm",
    md: "h-10 px-4 py-2",
    lg: "h-11 px-8 text-lg"
  };
  
  const classes = `${baseClasses} ${variantClasses[variant]} ${sizeClasses[size]} ${className}`;
  
  return (
    <button className={classes} {...props}>
      {children}
    </button>
  );
};
```

## Responsive Design Considerations

To ensure our UI works well on all devices, let's implement responsive design patterns:

```css
/* src/App.css */
@tailwind base;
@tailwind components;
@tailwind utilities;

/* Custom scrollbar styling */
::-webkit-scrollbar {
  width: 8px;
}

::-webkit-scrollbar-track {
  background: #f1f1f1;
}

::-webkit-scrollbar-thumb {
  background: #c5c5c5;
  border-radius: 4px;
}

::-webkit-scrollbar-thumb:hover {
  background: #a8a8a8;
}

/* Responsive adjustments */
@media (max-width: 768px) {
  .chat-container {
    flex-direction: column;
  }
  
  .sidebar {
    width: 100%;
    border-right: none;
    border-bottom: 1px solid #e5e7eb;
  }
}

/* Animation for loading states */
@keyframes pulse {
  0%, 100% {
    opacity: 1;
  }
  50% {
    opacity: 0.5;
  }
}

.loading {
  animation: pulse 1.5s ease-in-out infinite;
}

/* Truncate long text with ellipsis */
.line-clamp-2 {
  display: -webkit-box;
  -webkit-line-clamp: 2;
  -webkit-box-orient: vertical;
  overflow: hidden;
}
```

## Testing the Frontend

Let's create a test for our chat component:

```tsx
// src/__tests__/Chat.test.tsx
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import { QueryClient, QueryClientProvider } from '@tanstack/react-query';
import { AuthProvider } from 'better-auth-react';
import { Chat } from '../pages/Chat';

// Mock auth context
const mockAuthContext = {
  user: { id: '1', email: 'test@example.com', name: 'Test User' },
  isSignedIn: true,
  signOut: jest.fn(),
  signIn: jest.fn(),
  signUp: jest.fn(),
  updateUser: jest.fn()
};

// Mock auth provider
jest.mock('better-auth-react', () => ({
  useAuth: () => mockAuthContext,
  AuthProvider: ({ children }: { children: React.ReactNode }) => <>{children}</>,
}));

const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      retry: false,
    },
  },
});

const renderWithProviders = (component: React.ReactNode) => {
  return render(
    <QueryClientProvider client={queryClient}>
      <AuthProvider>
        {component}
      </AuthProvider>
    </QueryClientProvider>
  );
};

describe('Chat Component', () => {
  test('renders chat interface', () => {
    renderWithProviders(<Chat />);
    
    // Check that essential elements are present
    expect(screen.getByText('Book RAG Chatbot')).toBeInTheDocument();
    expect(screen.getByPlaceholderText('Ask a question about the book content...')).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /send/i })).toBeInTheDocument();
  });

  test('allows user to type and submit messages', async () => {
    renderWithProviders(<Chat />);
    
    const input = screen.getByPlaceholderText('Ask a question about the book content...');
    const submitButton = screen.getByRole('button', { name: /send/i });
    
    fireEvent.change(input, { target: { value: 'Hello, world!' } });
    fireEvent.click(submitButton);
    
    // Wait for the message to appear in the UI
    await waitFor(() => {
      expect(screen.getByText('Hello, world!')).toBeInTheDocument();
    });
  });
});
```

Continue to [Lesson 6: Integration and Deployment](./lesson-6-integration-deployment.md) to learn how to integrate all components and deploy the RAG chatbot system.