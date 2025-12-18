# Quickstart Guide: Integrate RAG Chatbot

**Branch**: `007-integrate-rag-chatbot` | **Date**: 2025-12-17 | **Spec**: specs/007-integrate-rag-chatbot/spec.md

## Overview

This guide provides instructions to quickly set up and run the integrated RAG chatbot system in a local development environment. It covers the necessary steps for both the FastAPI backend and the Docusaurus frontend.

## Prerequisites

-   **Python 3.11+**: For the FastAPI backend.
-   **Node.js 20+**: For the Docusaurus frontend.
-   **npm / yarn**: Node.js package managers.
-   **Git**: For cloning the repository.

## Backend Setup (FastAPI)

1.  **Navigate to the backend directory**:
    ```bash
    cd backend
    ```
2.  **Create and activate a Python virtual environment**:
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```
3.  **Install dependencies**:
    ```bash
    pip install -r requirements.txt
    ```
    *(Ensure `requirements.txt` includes `FastAPI`, `uvicorn`, `python-multipart` and any other RAG-specific dependencies like `qdrant-client`, `langchain` if not already present.)*
4.  **Run the FastAPI server**:
    ```bash
    uvicorn src.api.main:app --reload
    ```
    *(Adjust `src.api.main:app` if your FastAPI application entry point is different. Ensure CORS is correctly configured in your FastAPI app to allow requests from your Docusaurus frontend's local development URL, typically `http://localhost:3000`.)*

    The backend server should now be running, typically at `http://localhost:8000`.

## Frontend Setup (Docusaurus)

1.  **Navigate to the frontend directory**:
    ```bash
    cd frontend
    ```
2.  **Install dependencies**:
    ```bash
    npm install # or yarn install
    ```
3.  **Run the Docusaurus development server**:
    ```bash
    npm start # or yarn start
    ```
    The Docusaurus site should open in your browser, typically at `http://localhost:3000`.

## Testing the Integration

1.  Ensure both the FastAPI backend and Docusaurus frontend are running locally as described above.
2.  Access the Docusaurus frontend (e.g., `http://localhost:3000`).
3.  **Locate the Chatbot UI:** The chatbot component is integrated as a floating element at the bottom-right of the screen.
4.  **Verify General Questions:** Type a question into the chatbot's input field and click "Send". You should receive a "Dummy response." from the backend.
5.  **Verify Context-Restricted Questions:**
    *   Select some text on any page of the Docusaurus site.
    *   Type a question into the chatbot's input field and click "Send".
    *   To verify that the `selected_text` is being sent, open your browser's developer console (F12), go to the "Network" tab, and inspect the payload of the POST request to `http://localhost:8000/chat`. You should see `selected_text` containing the text you selected.
6.  **Verify Error Handling (Backend Unavailable):**
    *   Stop the FastAPI backend server (e.g., `Ctrl+C` in the backend terminal).
    *   Try sending a message via the chatbot. It should display a graceful error message like "Error: Could not get a response." without the frontend crashing.
    *   Restart the backend server.

**Note:** The backend currently returns a "Dummy response." for all queries. This will be replaced with actual RAG agent responses in subsequent development phases.

This quickstart guide should get you up and running with the RAG chatbot integration and verify its basic functionality.
