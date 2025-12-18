---
id: 2
title: Diagnose API and WebSocket Connection Failures
stage: general
date: 2025-12-17
surface: agent
model: gemini-1.5-flash
feature: none
branch: main
user: 
command: 
labels:
  - websocket
  - connection-refused
  - frontend
  - backend
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: []
tests_yaml: []
prompt_text: |
  installHook.js:1 Error sending message to chatbot API: TypeError: Failed to fetch
      at n (ajaxRequestInterceptor.ps.js:1:12912)
      at fetch (ajaxRequestInterceptor.ps.js:1:13985)
  overrideMethod	 @	installHook.js:1
  eval	 @	chatbotApi.ts:3
  Promise.then		
  sendMessage	 @	chatbotApi.ts:3
  eval	 @	index.tsx:18
  handleSendMessage	 @	index.tsx:24
  onKeyPress	 @	index.tsx:24
  <input>		
  Chatbot	 @	index.tsx:24
  <Chatbot>		
  LayoutWrapper	 @	index.tsx:3
  <Route>		
  ./node_modules/ @docusaurus/core/lib/client/App.js @	main.js:247
  __webpack_require__	 @	runtime~main.js:34
  fn	 @	runtime~main.js:333
  ./node_modules/ @docusaurus/core/lib/client/clientEntry.js @	main.js:311
  __webpack_require__	 @	runtime~main.js:34
  __webpack_exec__	 @	main.js:2136
  (anonymous)	 @	main.js:2137
  __webpack_require__.O	 @	runtime~main.js:77
  (anonymous)	 @	main.js:2138
  webpackJsonpCallback	 @	runtime~main.js:1355
  (anonymous)	 @	main.js:9
  WebSocketClient.js:19 WebSocket connection to 'ws://localhost:3001/ws' failed: 
  WebSocketClient	 @	WebSocketClient.js:19
  initSocket	 @	socket.js:36
  eval	 @	socket.js:63
  installHook.js:1 [webpack-dev-server] 
  Event {isTrusted: true, type: 'error', target: WebSocket, currentTarget: WebSocket, eventPhase: 2, …}
  overrideMethod	 @	installHook.js:1
  WebSocketClient.js:19 WebSocket connection to 'ws://localhost:3001/ws' failed: 
  WebSocketClient	 @	WebSocketClient.js:19
  initSocket	 @	socket.js:36
  eval	 @	socket.js:63
  installHook.js:1 [webpack-dev-server] 
  Event {isTrusted: true, type: 'error', target: WebSocket, currentTarget: WebSocket, eventPhase: 2, …}
  WebSocketClient.js:19 WebSocket connection to 'ws://localhost:3001/ws' failed: 
  installHook.js:1 [webpack-dev-server] 
  Event {isTrusted: true, type: 'error', target: WebSocket, currentTarget: WebSocket, eventPhase: 2, …}
  WebSocketClient.js:19 WebSocket connection to 'ws://localhost:3001/ws' failed: 
response_text: |
  The errors you are seeing ("TypeError: Failed to fetch" and "WebSocket connection to 'ws://localhost:3001/ws' failed") indicate that your frontend is unable to connect to its intended backend services.

  1.  **"TypeError: Failed to fetch" for 'http://localhost:8000/chat'**: This means your frontend cannot reach the chatbot API. Please ensure your backend server (the FastAPI application) is running and accessible on `http://localhost:8000`. You mentioned you are running the backend yourself, so please check its status and restart it if necessary.
  2.  **"WebSocket connection to 'ws://localhost:3001/ws' failed"**: This typically happens when the frontend's development server (e.g., from Webpack or Docusaurus) is not running, or its WebSocket feature is not active. Please ensure your frontend development server is properly started and running on `http://localhost:3001`. This WebSocket connection is often used for hot-reloading or live updates during development.
outcome: ""
evaluation: ""
---
