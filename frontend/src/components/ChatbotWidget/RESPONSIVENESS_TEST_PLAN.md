# Chatbot Widget Responsiveness Test Plan

This document outlines the approach for conducting responsiveness testing of the chatbot widget across various devices and screen sizes.

## Objective:
- Ensure the chatbot widget is usable and visually appealing on different screen sizes (mobile, tablet, desktop).
- Verify that UI elements (toggle button, chat window, input field, messages, citations) adapt gracefully.

## Test Environment:
-   **Browsers**: Chrome, Firefox, Safari (on macOS/iOS), Edge.
-   **Devices/Emulators**:
    -   Mobile: iPhone (various models), Android phone (various models).
    -   Tablet: iPad, Android tablet.
    -   Desktop: Standard monitor resolutions.
-   **Tools**: Browser developer tools (responsive design mode), actual physical devices.

## Test Cases:

### 1. Initial State (Chatbot Closed)
-   **Expected**: Toggle button is visible and correctly positioned (e.g., bottom-right) without overlapping critical content.

### 2. Open State (Chatbot Open)
-   **Expected**:
    -   Chat window opens with appropriate dimensions, fitting within the viewport.
    -   Chat window does not obstruct important page content significantly.
    -   Messages, input field, and header elements are clearly visible and usable.
    -   Scroll functionality within messages area works correctly.

### 3. Screen Size Variations:
-   **Desktop (>1024px width)**:
    -   Chat window maintains a consistent size and position.
-   **Tablet (768px - 1024px width)**:
    -   Chat window might adjust width/height, ensuring no overflow.
    -   Elements remain readable.
-   **Mobile (<768px width)**:
    -   Chat window should likely take up a larger portion of the screen (e.g., full width/height or a significant part).
    -   Text input and send button are accessible without covering the keyboard.
    -   Toggle button remains accessible.

### 4. Text Selection Button:
-   **Expected**: When text is selected, the "Answer based on selected text" button appears near the selection and is accessible, regardless of screen size. It should not be obscured by other UI elements.

## Procedure:
1.  Open the Docusaurus site in the target browser/device.
2.  Navigate to various pages to ensure consistent behavior.
3.  Perform text selection.
4.  Open and close the chatbot.
5.  Resize the browser window or rotate the device (if applicable) to observe responsiveness.
6.  Document any layout issues, overflows, unreadable text, or unresponsive elements.

## Acceptance Criteria:
-   No visual regressions (e.g., overlapping elements, truncated text).
-   All interactive elements are clickable and functional.
-   The chatbot widget provides a positive user experience across all tested devices/resolutions.
