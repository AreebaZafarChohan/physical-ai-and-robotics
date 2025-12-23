# Quickstart Guide: Signup & Signin Integration

**Date**: 2025-12-22
**Spec**: specs/010-better-auth-integration/spec.md
**Plan**: specs/010-better-auth-integration/plan.md

## Overview

This guide provides a quick overview of how to set up and run the Signup & Signin Integration feature. This feature enables user registration via email/password or OAuth (Google/GitHub), secure login, session management, and collection of user background information for personalization.

## Prerequisites

-   **Backend**: Python 3.11+ environment with FastAPI and necessary dependencies installed.
-   **Frontend**: Node.js 20+ environment with React and necessary dependencies installed.
-   **Database**: Access to a persistent data store (e.g., Neon Postgres) with connection details.
-   **Authentication Service**: BetterAuth SDK (or similar authentication service SDK) integrated into the backend.
-   **OAuth Providers**: Configured OAuth applications for Google and GitHub (client IDs and secrets).

## Setup Instructions

### 1. Database Configuration

Ensure your persistent data store (e.g., Neon Postgres) is accessible. Update backend configuration with the database connection string. The `User` table (and potentially `PersonalizationData` table) should be created as per the `data-model.md`.

### 2. Backend Setup

1.  **Clone the repository**:
    ```bash
    git clone <repository_url>
    cd <repository_name>/backend
    ```
2.  **Install dependencies**:
    ```bash
    pip install -r requirements.txt
    ```
3.  **Configure Environment Variables**:
    Create a `.env` file in the `backend` directory with the following (example values):
    ```env
    DATABASE_URL="postgresql://user:password@host:port/database"
    BETTERAUTH_CLIENT_ID="your_betterauth_client_id"
    BETTERAUTH_CLIENT_SECRET="your_betterauth_client_secret"
    GOOGLE_CLIENT_ID="your_google_client_id"
    GOOGLE_CLIENT_SECRET="your_google_client_secret"
    GITHUB_CLIENT_ID="your_github_client_id"
    GITHUB_CLIENT_SECRET="your_github_client_secret"
    SECRET_KEY="a_very_secret_key_for_jwt"
    ```
4.  **Run Migrations (if applicable)**:
    Apply any necessary database migrations to set up the `User` and `PersonalizationData` tables.
    ```bash
    # Example for Alembic with FastAPI
    alembic upgrade head
    ```
5.  **Start the Backend Server**:
    ```bash
    uvicorn main:app --reload
    ```
    The backend API should now be running, typically on `http://localhost:8000`.

### 3. Frontend Setup

1.  **Navigate to the frontend directory**:
    ```bash
    cd <repository_name>/frontend
    ```
2.  **Install dependencies**:
    ```bash
    npm install
    # or yarn install
    ```
3.  **Configure Environment Variables**:
    Create a `.env` file in the `frontend` directory with the following (example values):
    ```env
    REACT_APP_BACKEND_URL="http://localhost:8000/api/v1"
    ```
4.  **Start the Frontend Development Server**:
    ```bash
    npm start
    # or yarn start
    ```
    The frontend application should now be accessible in your browser, typically on `http://localhost:3000`.

## Testing the Feature

-   **User Registration**: Navigate to the signup page, attempt to register with email/password and via Google/GitHub OAuth. Verify successful account creation and login.
-   **User Login**: Navigate to the login page, attempt to log in with created credentials or OAuth. Verify successful authentication and session maintenance.
-   **Profile Update**: (Implicitly covered by signup data collection) Verify that software/hardware background data is correctly displayed in a user profile section (if implemented, otherwise verify directly in DB).
-   **API Endpoints**: Use a tool like Postman or Insomnia to test the `/signup`, `/login`, `/signup/oauth`, `/user/profile`, and `/user/personalization-data` endpoints.

This quickstart should get you up and running with the core functionality of the Signup & Signin Integration.
