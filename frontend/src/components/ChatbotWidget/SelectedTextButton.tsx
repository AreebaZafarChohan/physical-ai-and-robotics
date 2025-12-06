// frontend/src/components/ChatbotWidget/SelectedTextButton.tsx
import React from 'react';

interface SelectedTextButtonProps {
  onClick: () => void;
  isVisible: boolean;
  position: {
    top: number;
    left: number;
  };
}

const SelectedTextButton: React.FC<SelectedTextButtonProps> = ({ 
  onClick, 
  isVisible, 
  position 
}) => {
  if (!isVisible) return null;

  const buttonStyle: React.CSSProperties = {
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