export function getSelectedText(): string | undefined {
  if (typeof window === 'undefined') {
    return undefined; // Or throw an error, depending on desired behavior for SSR
  }

  const selection = window.getSelection();
  if (selection && selection.toString().length > 0) {
    return selection.toString();
  }
  return undefined;
}

// Optional: A function to listen for selection changes (for more dynamic updates if needed)
export function onTextSelectionChange(callback: (selectedText: string | undefined) => void) {
  if (typeof document === 'undefined') {
    return;
  }

  const handleSelectionChange = () => {
    callback(getSelectedText());
  };

  document.addEventListener('selectionchange', handleSelectionChange);

  // Return a cleanup function
  return () => {
    document.removeEventListener('selectionchange', handleSelectionChange);
  };
}
