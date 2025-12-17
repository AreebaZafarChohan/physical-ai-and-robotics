import React, { useState, useEffect } from 'react';
import { sendMessage } from '../../utils/chatbotApi'; // Import the API function
import wsClient from '../../utils/websocketClient'; // Import WebSocket client
import { getSelectedText } from '../../utils/textSelection'; // Import getSelectedText

const Chatbot: React.FC = () => {
  const [input, setInput] = useState<string>('');
  const [messages, setMessages] = useState<{ text: string; sender: 'user' | 'bot' }[]>([]);
  const [loading, setLoading] = useState<boolean>(false);
  const [useWebSocket, setUseWebSocket] = useState<boolean>(false); // Track if WebSocket is available
  const [showChatbot, setShowChatbot] = useState<boolean>(false); // State to control chatbot visibility

  const toggleChatbot = () => {
    setShowChatbot(!showChatbot);
  };

  // Initialize WebSocket connection on component mount
  useEffect(() => {
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
      if (wsClient.isConnected()) {
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

      if (useWebSocket && wsClient.isConnected()) {
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

  return (
    <>
      {showChatbot && (
        <div className="chatbot-container">
          <h3>Chatbot</h3>
          <div className="chat-history">
            {messages.map((msg, index) => (
              <p key={index} className={msg.sender === 'user' ? 'user-message' : 'bot-message'}>
                {msg.sender === 'user' ? 'You: ' : 'Bot: '}
                {msg.text}
              </p>
            ))}
            {loading && <p className="bot-message">Bot: Typing...</p>}
          </div>
          <div className="chat-input-area">
            <input
              type="text"
              value={input}
              onChange={handleInputChange}
              onKeyPress={(event) => {
                if (event.key === 'Enter' && !loading) {
                  handleSendMessage();
                }
              }}
              placeholder="Ask a question..."
              disabled={loading}
            />
            <button onClick={handleSendMessage} disabled={loading}>Send</button>
          </div>
        </div>
      )}
      <button className="chatbot-toggle-button" onClick={toggleChatbot}>
        {showChatbot ? 'X' : '💬'} {/* Placeholder for icon */}
      </button>
    </>
  );
};

export default Chatbot;