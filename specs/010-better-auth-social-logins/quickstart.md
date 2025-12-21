# Quickstart: User Authentication

This guide provides instructions for setting up and testing the user authentication feature locally.

## Prerequisites

- Python 3.11+
- Node.js 20+
- Docker
- A Better-Auth account and API keys

## Backend Setup

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/AreebaZafarChohan/physical-ai-and-robotics.git
    cd physical-ai-and-robotics
    ```

2.  **Set up the backend environment**:
    ```bash
    cd backend
    python -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt
    ```

3.  **Configure environment variables**:
    - Create a `.env` file in the `backend` directory.
    - Add your Better-Auth API keys and Neon Postgres connection string to the `.env` file:
      ```
      BETTER_AUTH_API_KEY=...
      BETTER_AUTH_API_SECRET=...
      NEON_POSTGRES_URL=...
      ```

4.  **Run the backend server**:
    ```bash
    uvicorn src.main:app --reload
    ```
    The backend will be running at `http://localhost:8000`.

## Frontend Setup

1.  **Set up the frontend environment**:
    ```bash
    cd ../frontend
    npm install
    ```

2.  **Run the frontend development server**:
    ```bash
    npm start
    ```
    The frontend will be running at `http://localhost:3000`.

## Testing the Flow

1.  **Sign up**:
    - Navigate to `http://localhost:3000/signup`.
    - Fill out the signup form and submit.
    - You should be logged in and redirected to the homepage.

2.  **Log in**:
    - Log out if you are already logged in.
    - Navigate to `http://localhost:3000/login`.
    - Enter your credentials and log in.
    - You should be successfully logged in.

3.  **Social Login**:
    - Click the "Sign in with Google" or "Sign in with GitHub" button.
    - You will be redirected to the respective provider for authentication.
    - Upon successful authentication, you will be logged in to the application.
