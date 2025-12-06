// frontend/src/utils/text_selection.js

export function getSelectedText() {
    let text = "";
    if (window.getSelection) {
        text = window.getSelection().toString();
    } else if (document.selection && document.selection.type != "Control") {
        text = document.selection.createRange().text;
    }
    return text.trim();
}

export function onTextSelectionChange(callback) {
    let timeoutId;
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
