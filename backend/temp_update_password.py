import os
from sqlmodel import create_engine, Session, select
from backend.src.models.user import User
from backend.src.utils.security import get_password_hash

# Database connection
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://user:password@localhost:5432/testdb")
engine = create_engine(DATABASE_URL)

def update_password(email: str, new_password: str):
    with Session(engine) as session:
        statement = select(User).where(User.email == email)
        user = session.exec(statement).first()
        if user:
            user.password_hash = get_password_hash(new_password)
            session.add(user)
            session.commit()
            session.refresh(user)
            print(f"Password for user {email} updated successfully.")
        else:
            print(f"User with email {email} not found.")

if __name__ == "__main__":
    # Replace with the user's email and a new password
    update_password("sarachohan755@gmail.com", "new_secure_password")
