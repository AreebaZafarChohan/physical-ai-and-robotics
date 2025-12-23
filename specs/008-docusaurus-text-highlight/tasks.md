### Phase 1: Setup
- [x] Create the directory structure for the new component: `frontend/src/components/HighlightMenu`.

### Phase 2: Core Component Development
- [x] Create the `HighlightMenu` React component (`index.tsx`).
- [x] Implement the logic to detect text selection using `window.getSelection()`.
- [x] Implement the UI for the context menu, which should appear near the selected text.
- [x] Style the context menu using CSS (`styles.css`).
- [x] Implement the "Copy Text" functionality using the Clipboard API.

### Phase 3: Integration with Chatbot
- [x] Identify the existing RAG chatbot component and its props.
- [x] Implement the "Ask from RoboX" functionality, which will:
    - Get the selected text.
    - Find the chatbot input element.
    - Programmatically set the input's value to the selected text.
    - Programmatically trigger the chatbot's submission (e.g., by simulating a click on the send button or submitting the form).

### Phase 4: Docusaurus Integration
- [x] Swizzle the Docusaurus `Layout` component to allow for customization.
- [x] Integrate the `HighlightMenu` component into the swizzled `Layout` component so it's available on all pages.

### Phase 5: Testing and Refinement
- [x] Test the feature across different browsers. (Requires manual testing)
- [x] Test with different lengths of selected text. (Requires manual testing)
- [x] Ensure the menu doesn't interfere with other UI elements. (Requires manual testing)
- [x] Refine the styling and positioning of the menu for a polished look and feel. (Requires manual testing)
