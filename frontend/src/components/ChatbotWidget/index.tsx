import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';
import { onTextSelectionChange } from '@site/src/utils/text_selection';
import SelectedTextButton from './SelectedTextButton';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useHistory } from '@docusaurus/router';

const ChatbotWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [currentMode, setCurrentMode] = useState<'book' | 'course'>('book');
  const [messageInput, setMessageInput] = useState('');
  const [messages, setMessages] = useState<{ sender: 'user' | 'bot'; text: string; citations?: string[] }[]>([
    { sender: 'bot', text: 'Hello! How can I help you with the textbook?' }
  ]);
  const [isBotTyping, setIsBotTyping] = useState(false);
  const [selectedText, setSelectedText] = useState<string>('');
  const [showSelectedTextButton, setShowSelectedTextButton] = useState(false);
  const [selectedTextButtonPosition, setSelectedTextButtonPosition] = useState({ top: 0, left: 0 });

  const chatMessagesRef = useRef<HTMLDivElement>(null);
  const {i18n} = useDocusaurusContext();
  const history = useHistory();
  const currentLocale = i18n.currentLocale;

  // Define backend URL (for development)
  const BACKEND_URL = 'http://localhost:8000/api/v1'; // Assuming backend runs on 8000 and has /api/v1 prefix

  useEffect(() => {
    const cleanup = onTextSelectionChange((text) => {
      setSelectedText(text);
      if (text && !isOpen) {
        const selection = window.getSelection();
        if (selection && selection.rangeCount > 0) {
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();
          setSelectedTextButtonPosition({
            top: window.scrollY + rect.top - 40,
            left: window.scrollX + rect.left + rect.width / 2 - 100,
          });
          setShowSelectedTextButton(true);
        }
      } else {
        setShowSelectedTextButton(false);
      }
    });
    return cleanup;
  }, [isOpen]);

  useEffect(() => {
    if (chatMessagesRef.current) {
      chatMessagesRef.current.scrollTop = chatMessagesRef.current.scrollHeight;
    }
  }, [messages, isBotTyping]);


  const toggleChat = () => {
    setIsOpen(!isOpen);
    setShowSelectedTextButton(false);
  };

  const simulateStreamingResponse = (fullResponse: string, citations?: string[]) => {
    setIsBotTyping(true);
    let currentBotResponse = '';
    const words = fullResponse.split(' ');
    let i = 0;

    const intervalId = setInterval(() => {
      if (i < words.length) {
        currentBotResponse += (i > 0 ? ' ' : '') + words[i];
        setMessages((prevMessages) => {
          const lastMessage = prevMessages[prevMessages.length - 1];
          if (lastMessage && lastMessage.sender === 'bot') {
            return [...prevMessages.slice(0, -1), { ...lastMessage, text: currentBotResponse, citations }];
          }
          return [...prevMessages, { sender: 'bot', text: currentBotResponse, citations }];
        });
        i++;
      } else {
        clearInterval(intervalId);
        setIsBotTyping(false);
      }
    }, 50);
  };

  const handleSendMessage = async (queryText: string, useSelectedText: boolean = false) => {
    if (!queryText.trim()) return;

    const newUserMessage = { sender: 'user', text: queryText };
    setMessages((prevMessages) => [...prevMessages, newUserMessage]);
    setMessageInput('');
    setIsBotTyping(true);
    setShowSelectedTextButton(false);

    // Add an empty bot message to be updated
    setMessages((prevMessages) => [...prevMessages, { sender: 'bot', text: '' }]);

    try {
      let endpoint = '';
      let payload: any = {};

      if (useSelectedText && selectedText) {
        endpoint = `${BACKEND_URL}/selected_text`;
        payload = {
          query: queryText,
          selected_text: selectedText,
          language: currentLocale, // Use current locale
          // user_id: 'some_user_id' // Optional
        };
      } else {
        endpoint = `${BACKEND_URL}/ask`;
        payload = {
          query: queryText,
          context: [],
          chat_history: messages.filter(m => m.sender !== 'bot').map(m => ({ role: m.sender, content: m.text })),
          language: currentLocale, // Use current locale
          mode: currentMode,
          // user_id: 'some_user_id' // Optional
        };
      }

      const response = await fetch(endpoint, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to fetch response from backend');
      }

      const data = await response.json();
      simulateStreamingResponse(data.answer, data.citation_links);

    } catch (error) {
      console.error('Error sending message to backend:', error);
      simulateStreamingResponse(`Error: ${error.message || 'Something went wrong.'}`);
    } finally {
      setIsBotTyping(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !isBotTyping) {
      handleSendMessage(messageInput);
    }
  };

  const handleAskSelectedText = () => {
    if (selectedText && !isBotTyping) {
      setIsOpen(true);
      handleSendMessage(messageInput || selectedText, true);
      setSelectedText('');
      setShowSelectedTextButton(false);
    }
  };

  const handleModeChange = (mode: 'book' | 'course') => {
    setCurrentMode(mode);
    setMessages([{ sender: 'bot', text: `Switched to "${mode}" mode. How can I help you?` }]);
  };

  const handleLanguageChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const newLocale = e.target.value;
    const path = window.location.pathname.replace(`/${i18n.currentLocale}/`, `/${newLocale}/`);
    history.push(path); // Docusaurus handles page reload for locale change
  };


  return (
    <>
      <SelectedTextButton
        isVisible={showSelectedTextButton && !isOpen && selectedText.length > 0}
        position={selectedTextButtonPosition}
        onClick={handleAskSelectedText}
      />
      <div className={styles.chatbotContainer}>
        <button className={styles.chatToggleButton} onClick={toggleChat}>
          {isOpen ? 'Close Chat' : 'Open Chat'}
        </button>

        {isOpen && (
          <div className={styles.chatWindow}>
            <div className={styles.chatHeader}>
              <h3>AI Assistant</h3>
              <div className={styles.headerControls}>
                <div className={styles.modeToggle}>
                  <button
                    className={`${styles.modeButton} ${currentMode === 'book' ? styles.activeMode : ''}`}
                    onClick={() => handleModeChange('book')}
                    disabled={isBotTyping}
                  >
                    Book
                  </button>
                  <button
                    className={`${styles.modeButton} ${currentMode === 'course' ? styles.activeMode : ''}`}
                    onClick={() => handleModeChange('course')}
                    disabled={isBotTyping}
                  >
                    Course
                  </button>
                </div>
                <select className={styles.languageSelect} onChange={handleLanguageChange} value={currentLocale} disabled={isBotTyping}>
                  {i18n.locales.map((locale) => (
                    <option key={locale} value={locale}>
                      {locale.toUpperCase()}
                    </option>
                  ))}
                </select>
              </div>
            </div>
            <div ref={chatMessagesRef} className={styles.chatMessages}>
              {messages.map((msg, index) => (
                <div key={index} className={msg.sender === 'bot' ? styles.botMessageContainer : styles.userMessageContainer}>
                  <p className={msg.sender === 'bot' ? styles.botMessage : styles.userMessage}>
                    {msg.text}
                  </p>
                  {msg.citations && msg.citations.length > 0 && msg.sender === 'bot' && (
                    <div className={styles.citations}>
                      <strong>Sources:</strong>
                      {msg.citations.map((citation, idx) => (
                        <a key={idx} href={citation} target="_blank" rel="noopener noreferrer" className={styles.citationLink}>
                          Source {idx + 1}
                        </a>
                      ))}
                    </div>
                  )}
                </div>
              ))}
              {isBotTyping && <p className={styles.botMessage + ' ' + styles.typingIndicator}>...</p>}
            </div>
            <div className={styles.chatInput}>
              <input
                type="text"
                placeholder="Ask a question..."
                value={messageInput}
                onChange={(e) => setMessageInput(e.target.value)}
                onKeyPress={handleKeyPress}
                disabled={isBotTyping}
              />
              <button onClick={handleSendMessage} disabled={isBotTyping}>Send</button>
            </div>
          </div>
        )}
      </div>
    </>
  );
};

export default ChatbotWidget;