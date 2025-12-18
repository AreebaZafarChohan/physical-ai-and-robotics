---
id: 3
title: Frontend API Port Mismatch and WebSocket Failure
stage: general
date: 2025-12-17
surface: agent
model: gemini-1.5-flash
feature: none
branch: main
user: 
command: 
labels:
  - frontend
  - backend
  - port-mismatch
  - websocket
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - frontend/src/utils/chatbotApi.ts
tests_yaml: []
prompt_text: |
  9000 pr chl rha he backend
response_text: |
  Your backend server is running on port `9000`, but your frontend is currently configured to connect to `http://localhost:8000`.

  To resolve this, please update the `CHAT_API_BASE_URL` in the file `frontend/src/utils/chatbotApi.ts` from `http://localhost:8000` to `http://localhost:9000`.

  After making this change, please restart your frontend development server for the changes to take effect. Ensure your backend is still running on port `9000`.

  If the WebSocket connection error (`WebSocket connection to 'ws://localhost:3001/ws' failed:`) persists after restarting your frontend, it might indicate a separate issue with your Docusaurus development server setup, but the primary API communication should be resolved by correcting the port.
outcome: ""
evaluation: ""
---
