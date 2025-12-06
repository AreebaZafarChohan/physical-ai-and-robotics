// frontend/src/components/ChatbotWidget/SelectedTextButton.js
import React from 'react';

const SelectedTextButton = ({ onClick, isVisible, position }) => {
  if (!isVisible) return null;

  const buttonStyle = {
    position: 'absolute',
    top: position.top + 'px',
    left: position.left + 'px',
    backgroundColor: '#3b82f6', // Blue color
    color: 'white',
    padding: '5px 10px',
    borderRadius: '5px',
    cursor: 'pointer',
    zIndex: 1001, // Above the chatbot
    border: 'none',
    fontSize: '0.9em',
    whiteSpace: 'nowrap',
  };

  return (
    <button style={buttonStyle} onClick={onClick}>
      Answer based on selected text
    </button>
  );
};

export default SelectedTextButton;
