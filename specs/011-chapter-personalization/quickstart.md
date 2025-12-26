# Quickstart: Chapter Content Personalization

## Overview
This guide provides instructions for setting up, running, and developing the chapter content personalization feature.

## Prerequisites
- Python 3.11+
- Node.js 18+ (for Docusaurus)
- PostgreSQL (or access to Neon Postgres)
- Redis server
- Existing project dependencies from the main repository

## Setup Instructions

### 1. Environment Configuration
```bash
# Clone the repository (if not already done)
git clone <repository-url>
cd Physical_AI_And_Robotics

# Create a virtual environment for the backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install backend dependencies
pip install -r requirements.txt

# Install frontend dependencies (if not already done)
cd frontend
npm install
```

### 2. Environment Variables
Create a `.env` file in the backend directory with the following variables:

```env
DATABASE_URL="postgresql://user:password@localhost:5432/personalization_db"
REDIS_URL="redis://localhost:6379"
SECRET_KEY="your-super-secret-key-here"
ALGORITHM="HS256"
ACCESS_TOKEN_EXPIRE_MINUTES=30
```

### 3. Database Setup
```bash
# Run database migrations
cd backend
python -m src.db.migrate

# Initialize with default personalization rules (optional)
python -m src.db.seed
```

### 4. Running the Services

#### Backend Service
```bash
cd backend
# Activate virtual environment if not already done
source venv/bin/activate
# Run the FastAPI server
uvicorn src.api.main:app --reload --port 8000
```

#### Frontend Service
```bash
cd frontend
npm start
```

The Docusaurus site will be available at `http://localhost:3000`.

## Development Workflow

### Adding Personalization Markers to Content
In your chapter markdown files, add special markers where you want content to be personalized:

```
This is generic content.

{{personalize:beginner}}
This content will be shown only to beginner users.
{{personalize:end}}

{{personalize:python}}
This content is specific to Python developers.
{{personalize:end}}

{{personalize:raspberry-pi}}
This content is specific to Raspberry Pi users.
{{personalize:end}}
```

### Creating Personalization Rules
1. Access the admin interface at `/admin/personalization` (when implemented)
2. Create a new rule with:
   - Content path (e.g., `/module-1/chapter-1`)
   - User profile criteria (e.g., `{"software_background": ["Python"]}`)
   - Content variants for different profiles
   - Priority level (lower numbers = higher priority)

### Testing the Feature
1. Ensure you have a user account with profile information (software/hardware background)
2. Navigate to a chapter page
3. Verify the "Personalize Content" button appears
4. Click the button and verify content changes based on your profile
5. Check browser developer tools to confirm API calls are made and content is replaced

### API Endpoints
- `GET /api/personalize/{chapter_path}` - Fetch personalized content
- `POST /api/personalize/preview` - Preview content with custom profile
- `GET /api/personalize/rules` - List personalization rules (admin)
- `PUT /api/personalize/rules/{id}` - Update personalization rule (admin)

## Running Tests
```bash
# Backend tests
cd backend
python -m pytest tests/

# Frontend tests
cd frontend
npm test
```

## Code Structure
```
backend/
├── src/
│   ├── models/           # Database models (Pydantic/SQLAlchemy)
│   ├── services/         # Business logic
│   │   ├── personalization_service.py
│   │   ├── user_service.py
│   │   └── content_service.py
│   └── api/
│       ├── routes/
│       │   ├── personalization.py
│       │   └── user_profile.py
│       └── main.py
└── tests/
    ├── unit/
    └── integration/

frontend/
├── src/
│   ├── components/
│   │   ├── PersonalizeContentButton.tsx
│   │   └── PersonalizedContent.tsx
│   ├── services/
│   │   └── api.ts
│   └── utils/
│       └── contentParser.ts
└── tests/
```

## Troubleshooting
- Q: Personalization button doesn't appear
  A: Verify you're logged in and have profile information set
  
- Q: Content doesn't update after clicking button
  A: Check browser console for errors and verify backend API is accessible
  
- Q: API returns 401 Unauthorized
  A: Verify your JWT token is valid and properly formatted in the Authorization header

## Next Steps
1. Review the [data model](data-model.md) to understand the database structure
2. Examine the [API contracts](contracts/personalization-api.yaml) for detailed endpoint specifications
3. Check out the [tasks](tasks.md) document for implementation details