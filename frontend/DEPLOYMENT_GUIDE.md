# Frontend Deployment Guide (Docusaurus)

This document outlines the steps for deploying the Docusaurus frontend to a production environment, typically a static hosting service like GitHub Pages.

## Objective:
-   Successfully deploy the Docusaurus-based chatbot frontend to a public URL.
-   Ensure the deployed site is accessible, performant, and correctly configured.

## Deployment Process (GitHub Pages Example):

### Prerequisites:
-   A GitHub repository for the project.
-   `frontend/docusaurus.config.ts` configured for GitHub Pages deployment (already done in T052).
-   Node.js and npm/yarn installed.

### Steps:

1.  **Build the Docusaurus Project**:
    Navigate to the `frontend/` directory and run the build command:
    ```bash
    cd frontend/
    npm install # Ensure dependencies are installed
    npm run build
    ```
    This command generates static content into the `build` directory.

2.  **Deploy to GitHub Pages**:
    Docusaurus provides a convenient `deploy` command that handles pushing the `build` directory content to the `gh-pages` branch of your repository, which GitHub Pages then serves.
    ```bash
    npm run deploy
    ```
    Alternatively, you can manually push the `build` folder content to the `gh-pages` branch.

3.  **Configure GitHub Pages**:
    -   Go to your GitHub repository settings.
    -   Navigate to "Pages" under the "Code and automation" section.
    -   Select the `gh-pages` branch as the source and `/ (root)` as the folder.
    -   Save the changes.
    -   Your site should be available at `https://<YOUR_GITHUB_USERNAME>.github.io/<YOUR_REPO_NAME>/`.

### Post-Deployment Verification:
-   Access the deployed URL in a browser.
-   Verify all pages load correctly.
-   Check console for any errors.
-   Ensure the chatbot widget is visible and functional.
-   Test responsiveness across different devices.

### Troubleshooting:
-   If deployment fails, check the GitHub Actions logs.
-   Ensure `baseUrl` and `url` in `docusaurus.config.ts` are correct.
-   Clear browser cache if changes aren't reflected.
```
Let's create the file `frontend/DEPLOYMENT_GUIDE.md`.