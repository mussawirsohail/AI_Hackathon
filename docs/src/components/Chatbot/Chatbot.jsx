import React, { useState, useRef, useEffect } from 'react';
import {
  Box,
  Card,
  CardContent,
  TextField,
  Button,
  Typography,
  Divider,
  Paper,
  Avatar,
  IconButton,
  InputAdornment,
  Fab,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Chip,
  Badge
} from '@mui/material';
import SendIcon from '@mui/icons-material/Send';
import SmartToyIcon from '@mui/icons-material/SmartToy';
import PersonIcon from '@mui/icons-material/Person';
import CloseIcon from '@mui/icons-material/Close';
import ChatIcon from '@mui/icons-material/Chat';
import { styled } from '@mui/material/styles';
import { chatAPI } from './api';

const Chatbot = () => {
  // State for chat window visibility
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [hasUnreadMessage, setHasUnreadMessage] = useState(false);

  // State for chat
  const [messages, setMessages] = useState([
    {
      id: '1',
      text: 'Hello! I\'m your Physical AI & Humanoid Robotics Assistant. I can answer questions about ROS 2, Gazebo/Unity simulation, NVIDIA Isaac, Vision-Language-Action systems, and more. Ask me anything about Physical AI & Humanoid Robotics!',
      sender: 'bot',
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const messagesEndRef = useRef(null);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Mark messages as read when chat opens
  useEffect(() => {
    if (isChatOpen && hasUnreadMessage) {
      setHasUnreadMessage(false);
    }
  }, [isChatOpen, hasUnreadMessage]);

  // Toggle chat window
  const toggleChat = () => {
    setIsChatOpen(!isChatOpen);
  };

  // Send message to chatbot
  const handleSendMessage = async () => {
    if (inputValue.trim() === '') return;

    // Add user message
    const userMessage = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the API service
      const response = await chatAPI.sendMessage(inputValue);

      const botMessage = {
        id: (Date.now() + 1).toString(),
        text: response.response,
        sender: 'bot',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
      setHasUnreadMessage(true);
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message to chat
      const errorMessage = {
        id: (Date.now() + 1).toString(),
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
      setHasUnreadMessage(true);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle key press (Enter to send message)
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Close chat
  const handleClose = () => {
    setIsChatOpen(false);
  };

  return (
    <div className="chatbot-container">
      {/* Chat tab button positioned at bottom right */}
      <div
        className={`chatbot-fab ${hasUnreadMessage && !isChatOpen ? 'unread' : ''}`}
        onClick={toggleChat}
        aria-label={isChatOpen ? "Close chat" : "Open chat"}
      >
        <ChatIcon />
      </div>

      {/* Chat window with custom styling */}
      <div className={`chat-window ${isChatOpen ? 'open' : ''}`}>
        <div className="chat-header">
          <div className="chat-title">
            <SmartToyIcon />
            <span>Physical AI Assistant</span>
          </div>
          <button className="close-button" onClick={handleClose} aria-label="close">
            <CloseIcon />
          </button>
        </div>

        <div className="messages-container">
          {/* Example questions */}
          {messages.length <= 1 && (
            <div className="example-questions">
              <div className="example-questions-title">Try asking:</div>
              <div className="example-question-chips">
                {[
                  'What is ROS 2?',
                  'How does Gazebo simulate physics?',
                  'Explain Isaac ROS VSLAM',
                  'How to create a humanoid URDF model?',
                  'What is the VLA framework?',
                  'How do I make a robot understand voice commands?'
                ].map((question, idx) => (
                  <div
                    key={idx}
                    className="example-question-chip"
                    onClick={() => setInputValue(question)}
                  >
                    {question}
                  </div>
                ))}
              </div>
            </div>
          )}

          {/* Messages container */}
          {messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.sender}`}
            >
              <div className="avatar">
                {message.sender === 'user' ? <PersonIcon fontSize="small" /> : <SmartToyIcon fontSize="small" />}
              </div>
              <div className={`message-content ${message.sender}`}>
                <Typography variant="body2" sx={{ wordWrap: 'break-word' }}>{message.text}</Typography>
                <Typography
                  variant="caption"
                  className="message-timestamp"
                >
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </Typography>
              </div>
            </div>
          ))}
          {isLoading && (
            <div className="message bot">
              <div className="avatar bot">
                <SmartToyIcon fontSize="small" />
              </div>
              <div className="message-content bot">
                <div className="thinking-indicator">
                  <span>Thinking</span>
                  <div className="typing-indicator">
                    <div className="typing-dot"></div>
                    <div className="typing-dot"></div>
                    <div className="typing-dot"></div>
                  </div>
                </div>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <Divider />

        {/* Input area */}
        <div className="input-area">
          <TextField
            fullWidth
            variant="outlined"
            placeholder="Ask a question about Physical AI & Robotics..."
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            disabled={isLoading}
            multiline
            maxRows={4}
            className="message-input"
            InputProps={{
              sx: {
                borderRadius: '24px',
                backgroundColor: '#2d2d2d',
                color: 'white',
                '& .MuiOutlinedInput-notchedOutline': {
                  borderColor: '#444',
                },
                '&:hover .MuiOutlinedInput-notchedOutline': {
                  borderColor: '#6a1b9a',
                },
                '&.Mui-focused .MuiOutlinedInput-notchedOutline': {
                  borderColor: '#9c27b0',
                }
              },
              endAdornment: (
                <InputAdornment position="end">
                  <button
                    onClick={handleSendMessage}
                    disabled={isLoading || inputValue.trim() === ''}
                    className="send-button"
                    aria-label="send"
                  >
                    <SendIcon fontSize="small" />
                  </button>
                </InputAdornment>
              ),
            }}
          />
        </div>
      </div>
    </div>
  );
};

export default Chatbot;