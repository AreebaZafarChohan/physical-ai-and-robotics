# Physical AI & Humanoid Robotics - Documentation

This folder contains root-level documentation for the Physical AI & Humanoid Robotics project.

## Project Structure

```
Physical_AI_And_Robotics/
├── frontend/          # Docusaurus frontend application
├── backend/           # FastAPI backend service
├── docs/             # Root documentation (this folder)
├── specs/            # Project specifications
└── history/          # Development history and prompts
```

## Quick Links

- [Frontend Documentation](../frontend/README.md)
- [Backend Documentation](../backend/README.md)
- [Project Specifications](../specs/)

## Overview

This project provides an interactive, multilingual textbook platform for learning:

- ROS 2 fundamentals
- Digital Twin simulation
- NVIDIA Isaac robotics
- Vision-Language-Action (VLA) systems
- Humanoid robotics applications

## Tech Stack

**Frontend:**
- Docusaurus v3
- React + TypeScript
- i18n (English, Urdu, Arabic)
- Custom authentication

**Backend:**
- FastAPI
- SQLAlchemy
- PostgreSQL
- JWT authentication

## Getting Started

1. **Frontend Setup:**
   ```bash
   cd frontend
   npm install
   npm start
   ```

2. **Backend Setup:**
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   uvicorn src.main:app --reload
   ```

## Contributing

Please refer to the project specifications in the `specs/` folder before making changes.

## License

This project is open-source and available under the MIT License.
