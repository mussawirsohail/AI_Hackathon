import React from 'react';
import { Container, Typography } from '@mui/material';
import Chatbot from '../components/Chatbot/Chatbot';

const ChatPage = () => {
  return (
    <Container maxWidth="lg" sx={{ py: 4 }}>
      <Typography variant="h2" component="h1" gutterBottom align="center">
        RAG Chatbot for Book Content
      </Typography>
      <Typography variant="h5" component="h2" gutterBottom align="center" color="text.secondary">
        Ask questions about the book content and get AI-powered answers
      </Typography>
      
      <Chatbot />
    </Container>
  );
};

export default ChatPage;