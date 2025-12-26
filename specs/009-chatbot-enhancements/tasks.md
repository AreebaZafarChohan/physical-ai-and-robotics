### Phase 1: Welcome Screen
- [ ] Create a `WelcomeScreen` component in `frontend/src/components/WelcomeScreen`.
- [ ] The component should display the specified welcome message and a "Start Chatting" button.
- [ ] In the main chat component, use state to show the `WelcomeScreen` initially.
- [ ] On "Start Chatting" button click, hide the `WelcomeScreen` and show the chat interface.
- [ ] The first message from the assistant "RoboX is ready..." should be displayed after the chat starts.

### Phase 2: Authentication Backend
- [ ] Create a new FastAPI endpoint for user signup.
- [ ] Create a new FastAPI endpoint for user login.
- [ ] These endpoints should handle user creation and authentication. For now, a mock implementation is sufficient.
- [ ] On successful login, the endpoint should return a token and user information.

### Phase 3: Authentication Frontend
- [ ] Create a React AuthContext to manage authentication state (`userId`, `isAuthenticated`).
- [ ] Wire the existing login and signup UI components to the new backend endpoints.
- [ ] On successful login, update the AuthContext with the user's information.
- [ ] The `userId` and `isAuthenticated` status should be available to the chatbot component.

### Phase 4: User Memory (Backend)
- [ ] Create a table schema for `users` in Neon DB.
- [ ] Create a table schema for `user_memory` in Neon DB, with a foreign key to the `users` table.
- [ ] In the chat endpoint, check if the user is authenticated.
- [ ] If the user is authenticated, check incoming messages for explicit personal information.
- [ ] If personal information is found, save it to the `user_memory` table.
- [ ] Before generating a response, if the user is authenticated, retrieve their memory from the database.
- [ ] Use the retrieved memory to personalize the response (e.g., greeting).

### Phase 5: User Memory (Frontend)
- [ ] Pass the `isAuthenticated` status and `userId` to the backend when sending a chat message.

### Phase 6: Highlighted Text
- [ ] Ensure the existing "Ask from RoboX" functionality for highlighted text is working as expected with the new enhancements.

### Phase 7: Final Touches
- [ ] Ensure guest users do not have memory enabled.
- [ ] Review the implementation to ensure no breaking changes were introduced.
