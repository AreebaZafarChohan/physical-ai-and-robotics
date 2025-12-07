from sqlmodel import create_engine, Session
import os

DATABASE_URL = os.getenv("NEON_DB_URL")
if not DATABASE_URL:
    raise ValueError("NEON_DB_URL environment variable is not set.")

engine = create_engine(DATABASE_URL, echo=True)

def create_db_and_tables():
    # SQLModel.metadata.create_all(engine)
    # This will be called in a separate script/migration tool
    # For now, just ensure engine creation is fine.
    pass

def get_session():
    with Session(engine) as session:
        yield session
