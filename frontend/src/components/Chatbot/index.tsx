import React, { useState, useEffect, useRef } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import './chatbot.css';

import { sendMessage } from '../../utils/chatbotApi'; // Import the API function
import wsClient from '../../utils/websocketClient'; // Import WebSocket client
import { getSelectedText } from '../../utils/textSelection'; // Import getSelectedText

const Chatbot: React.FC = () => {
  const [input, setInput] = useState<string>('');
  const [messages, setMessages] = useState<{ text: string; sender: 'user' | 'bot' }[]>([]);
  const [loading, setLoading] = useState<boolean>(false);
  const [useWebSocket, setUseWebSocket] = useState<boolean>(false); // Track if WebSocket is available
  const [showChatbot, setShowChatbot] = useState<boolean>(false); // State to control chatbot visibility
  const chatHistoryRef = useRef<HTMLDivElement>(null);

  const toggleChatbot = () => {
    setShowChatbot(!showChatbot);
  };

  // Clear chat history
  const clearChatHistory = () => {
    setMessages([]);
  };

  // Scroll to bottom of chat history
  useEffect(() => {
    if (chatHistoryRef.current) {
      chatHistoryRef.current.scrollTop = chatHistoryRef.current.scrollHeight;
    }
  }, [messages, loading]);

  // Initialize WebSocket connection on component mount
  useEffect(() => {
    // Only initialize WebSocket if it's available
    if (!wsClient) {
      console.log('WebSocket client is not available, using HTTP API only');
      setUseWebSocket(false);
      return;
    }

    const initWebSocket = async () => {
      try {
        await wsClient.connect();
        setUseWebSocket(true);
        console.log('WebSocket connected successfully');
      } catch (error) {
        console.log('WebSocket connection failed, falling back to HTTP API:', error);
        setUseWebSocket(false);
      }
    };

    // Try to connect to WebSocket, but don't block the UI if it fails
    setTimeout(() => {
      initWebSocket().catch(() => {
        // If WebSocket connection fails, we still want to use HTTP API
        setUseWebSocket(false);
        console.log('Continuing with HTTP API only');
      });
    }, 1000); // Small delay to allow page to load first

    // Cleanup WebSocket connection on unmount
    return () => {
      if (wsClient && wsClient.isConnected()) {
        wsClient.close();
      }
    };
  }, []);

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setInput(event.target.value);
  };

  const handleSendMessage = async () => {
    if (input.trim()) {
      const userMessage = input;
      const selectedText = getSelectedText(); // Get selected text
      setMessages((prevMessages) => [...prevMessages, { text: userMessage, sender: 'user' }]);
      setInput('');
      setLoading(true);

      if (useWebSocket && wsClient && wsClient.isConnected()) {
        // Use WebSocket for real-time communication
        // Set up a temporary message handler for this specific response
        const messageHandler = (data: any) => {
          if (data.response) {
            setMessages((prevMessages) => [...prevMessages, { text: data.response, sender: 'bot' }]);
            setLoading(false);
            // Remove this specific handler after receiving the response
            wsClient.removeMessageHandler(messageHandler);
          }
        };

        // Add the message handler
        wsClient.addMessageHandler(messageHandler);

        // Send the message via WebSocket
        wsClient.send({
          query: userMessage,
          selected_text: selectedText
        });
      } else {
        // Use HTTP API as fallback
        try {
          const botResponse = await sendMessage(userMessage, selectedText); // Pass selected text
          setMessages((prevMessages) => [...prevMessages, { text: botResponse, sender: 'bot' }]);
        } catch (error) {
          setMessages((prevMessages) => [...prevMessages, { text: 'Error: Could not get a response.', sender: 'bot' }]);
        } finally {
          setLoading(false);
        }
      }
    }
  };

  const handleKeyPress = (event: React.KeyboardEvent<HTMLInputElement>) => {
    if (event.key === 'Enter' && !loading) {
      handleSendMessage();
    }
  };

  return (
    <>
      {showChatbot && (
        <motion.div
          className="chatbot-container chatbot-wrapper"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          exit={{ opacity: 0, y: 20 }}
          transition={{ duration: 0.3 }}
        >
          <div className="chat-header">
            <h3 className="chat-title">Chatbot</h3>
            <button className="clear-history-btn" onClick={clearChatHistory} title="Clear chat history">
              🗑️
            </button>
          </div>

          <div className="chat-history" ref={chatHistoryRef}>
            <AnimatePresence>
              {messages.map((msg, index) => (
                <motion.div
                  key={index}
                  className={`message ${msg.sender}-message`}
                  initial={{ opacity: 0, y: 10 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ duration: 0.3, delay: index * 0.05 }}
                >
                  {msg.text}
                </motion.div>
              ))}

              {loading && (
                <motion.div
                  className="typing-indicator"
                  initial={{ opacity: 0 }}
                  animate={{ opacity: 1 }}
                  transition={{ duration: 0.3 }}
                >
                  <span>Bot: </span>
                  <span className="dot"></span>
                  <span className="dot"></span>
                  <span className="dot"></span>
                </motion.div>
              )}
            </AnimatePresence>
          </div>

          <div className="chat-input-area">
            <input
              type="text"
              value={input}
              onChange={handleInputChange}
              onKeyPress={handleKeyPress}
              className="chat-input"
              placeholder="Ask a question..."
              disabled={loading}
              autoFocus
            />
            <button
              className="send-button"
              onClick={handleSendMessage}
              disabled={loading}
              title="Send message"
            >
              ➤
            </button>
          </div>
        </motion.div>
      )}
      <button className="chatbot-toggle-button" onClick={toggleChatbot}>
        {showChatbot ? '✕' : '💬'}
      </button>
    </>
  );
};

export default Chatbot;