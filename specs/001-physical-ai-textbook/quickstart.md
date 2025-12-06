# Quickstart: Building the Textbook

**Date**: 2025-12-06
**Feature**: Physical AI & Humanoid Robotics Textbook

This guide provides instructions on how to set up the development environment and build the textbook website locally.

## Prerequisites

-   [Node.js](https://nodejs.org/) (v20 or later)
-   [Yarn](https://yarnpkg.com/) (or npm)
-   [Git](https://git-scm.com/)

## Setup

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/AreebaZafarChohan/physical-ai-and-robotics.git
    cd physical-ai-and-robotics
    ```

2.  **Install dependencies**:
    ```bash
    yarn install
    ```

## Running the Development Server

To start a local development server with hot-reloading:

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server. The site will be available at `http://localhost:3000`.

## Building the Static Site

To generate a static production build of the website:

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static content hosting service. For this project, the `build` directory is what gets deployed to GitHub Pages.
