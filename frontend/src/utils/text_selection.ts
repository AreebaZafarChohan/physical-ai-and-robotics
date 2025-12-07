// frontend/src/utils/text_selection.ts

export function getSelectedText(): string {
  let text = "";
  if (window.getSelection) {
    text = window.getSelection()!.toString();
  } else if ((document as any).selection && (document as any).selection.type != "Control") {
    text = (document as any).selection.createRange().text;
  }
  return text.trim();
}

export function onTextSelectionChange(callback: (selectedText: string) => void): () => void {
  let timeoutId: NodeJS.Timeout;
  const handler = () => {
    clearTimeout(timeoutId);
    timeoutId = setTimeout(() => {
      const selectedText = getSelectedText();
      callback(selectedText);
    }, 200); // Debounce to prevent too frequent calls
  };

  document.addEventListener("mouseup", handler);
  document.addEventListener("keyup", handler);
  // Return a cleanup function
  return () => {
    document.removeEventListener("mouseup", handler);
    document.removeEventListener("keyup", handler);
  };
}