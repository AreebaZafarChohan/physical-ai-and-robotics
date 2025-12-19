import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';

const HighlightMenu = () => {
  const [selectedText, setSelectedText] = useState('');
  const [menuPosition, setMenuPosition] = useState({ top: 0, left: 0, visible: false });
  const menuRef = useRef(null);

  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        setSelectedText(text);
        setMenuPosition({
          top: rect.top + window.scrollY - 40,
          left: rect.left + window.scrollX + rect.width / 2,
          visible: true,
        });
      } else {
        setMenuPosition({ ...menuPosition, visible: false });
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  const handleCopy = () => {
    navigator.clipboard.writeText(selectedText);
    setMenuPosition({ ...menuPosition, visible: false });
  };

  const handleAskRoboX = () => {
    setMenuPosition({ ...menuPosition, visible: false });

    const inputElement = document.getElementById('chatbot-input') as HTMLInputElement;
    const sendButton = document.getElementById('chatbot-send-button');

    if (inputElement && sendButton) {
      const nativeInputValueSetter = Object.getOwnPropertyDescriptor(window.HTMLInputElement.prototype, 'value').set;
      nativeInputValueSetter.call(inputElement, selectedText);

      const event = new Event('input', { bubbles: true });
      inputElement.dispatchEvent(event);

      sendButton.click();
    } else {
      console.error('Chatbot input or send button not found');
    }
  };

  if (!menuPosition.visible) {
    return null;
  }

  return (
    <div
      ref={menuRef}
      className={styles.highlightMenu}
      style={{ top: `${menuPosition.top}px`, left: `${menuPosition.left}px`}}
    >
      <button onClick={handleCopy}>Copy Text</button>
      <button onClick={handleAskRoboX}>Ask from RoboX</button>
    </div>
  );
};

export default HighlightMenu;
