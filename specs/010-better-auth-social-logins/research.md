# Research: Better-Auth Integration

This document outlines the research conducted to resolve open questions from the implementation plan.

## 1. Performance Goals & Scale

**Decision**:
- **Performance Goals**: The authentication endpoints (signup, signin) should have a p95 latency of less than 300ms under normal load.
- **Scale/Scope**: The system will be designed to support up to 1,000 concurrent users and a total of 100,000 user accounts initially.

**Rationale**:
- A 300ms latency is a reasonable starting point for a good user experience for authentication.
- The user load estimates are based on typical initial adoption for a niche educational platform. These numbers can be revised as user growth is observed.

**Alternatives considered**:
- Stricter latency goals (e.g., <100ms) were considered but deemed unnecessary for the initial implementation, as they would require more complex infrastructure and optimization.

## 2. Better-Auth Integration with FastAPI

**Decision**:
- The integration will be done using standard REST API calls from the FastAPI backend to the Better-Auth service.
- The FastAPI backend will act as a proxy, handling requests from the frontend and communicating with Better-Auth.
- API keys for Better-Auth will be stored securely using environment variables.

**Rationale**:
- A proxy architecture decouples the frontend from the authentication provider, allowing for more flexibility and security.
- Using environment variables is a standard and secure way to manage API keys.

**Alternatives considered**:
- Direct integration from the frontend to Better-Auth was considered but rejected due to security concerns (exposing API keys) and lack of flexibility.

## 3. Better-Auth Integration with React/Docusaurus

**Decision**:
- The React frontend will communicate with the FastAPI backend for all authentication-related actions.
- A React context will be used to manage the user's authentication state (e.g., logged in, user profile).
- Social login buttons will trigger the backend API, which will then redirect the user to the appropriate Better-Auth social login page.

**Rationale**:
- This approach maintains a clean separation of concerns between the frontend and backend.
- Using a React context is a standard way to manage global state in a React application.

**Alternatives considered**:
- Using the Better-Auth frontend SDK directly was considered, but this would create a tighter coupling with the authentication provider and reduce the backend's control over the authentication flow.
