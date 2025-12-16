// API service for the Physical AI & Humanoid Robotics RAG chatbot
// This will connect to the backend API that's specialized for Physical AI & Humanoid Robotics content

// For Docusaurus, we'll define the base URL directly
// You can change this to point to your deployed backend
const API_BASE_URL = 'https://mussawirsoomro5-physical-ai.hf.space/api/v1';

// Create a request helper function
const request = async (endpoint, options = {}) => {
  const url = `${API_BASE_URL}${endpoint}`;

  const config = {
    headers: {
      'Content-Type': 'application/json',
      ...options.headers,
    },
    ...options,
  };

  try {
    const response = await fetch(url, config);

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error(`API request failed: ${url}`, error);
    throw error;
  }
};

// Chat API calls
export const chatAPI = {
  sendMessage: (message, contextText) => {
    return request('/chat', {
      method: 'POST',
      body: JSON.stringify({
        message,
        context_text: contextText || null
      })
    });
  }
};