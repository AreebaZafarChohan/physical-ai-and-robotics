## Tech Stack
- Docusaurus
- React
- TypeScript/JavaScript
- FastAPI (existing backend)

## Architecture
The feature will be implemented as a frontend-only React component.
- The component will listen for text selection events on the page.
- When text is selected, it will display a small, non-intrusive menu near the selected text.
- The menu will have two buttons: "Copy Text" and "Ask from RoboX".
- "Copy Text" will use the browser's Clipboard API to copy the selected text.
- "Ask from RoboX" will programmatically interact with the existing RAG chatbot component, populating its input and triggering a submission.
- The component will be implemented in TypeScript and styled with CSS.

## File Structure
- **New Component**: `frontend/src/components/HighlightMenu/index.tsx`
- **Component Styles**: `frontend/src/components/HighlightMenu/styles.css`
- **Integration**: The `HighlightMenu` component will be added to the main layout of the Docusaurus site, likely by swizzling a theme component like `Layout` and adding the new component there. The target for this integration will be `frontend/src/theme/Layout.tsx`.
