from typing import Optional, Dict, List
from sqlmodel import Session, select
from backend.src.models.user import User
from backend.src.models.personalization import PersonalizationData
from backend.src.utils.security import get_password_hash, verify_password
from backend.src.utils.errors import UserAlreadyExistsException, InvalidCredentialsException

class UserService:
    def __init__(self, session: Session):
        self.session = session

    def get_user_by_email(self, email: str) -> Optional[User]:
        statement = select(User).where(User.email == email)
        return self.session.exec(statement).first()

    def get_user_by_id(self, user_id: int) -> Optional[User]:
        return self.session.get(User, user_id)

    def authenticate_user(self, email: str, password: str) -> Optional[User]:
        user = self.get_user_by_email(email)
        if not user:
            return None
        if not verify_password(password, user.password_hash):
            return None
        return user

    def create_user_with_password(self, email: str, password: str,
                                  software_background: Optional[List[str]] = None,
                                  hardware_background: Optional[List[str]] = None) -> User:
        if self.get_user_by_email(email):
            raise UserAlreadyExistsException()

        hashed_password = get_password_hash(password)
        user = User(email=email, password_hash=hashed_password)
        self.session.add(user)
        self.session.commit()
        self.session.refresh(user)

        if software_background or hardware_background:
            personalization_data = PersonalizationData(
                user_id=user.id,
                software_background=software_background or [],
                hardware_background=hardware_background or []
            )
            self.session.add(personalization_data)
            self.session.commit()
            self.session.refresh(personalization_data)
            user.personalization_data = personalization_data # Link personalization data to user

        return user

    def create_user_with_oauth(self, email: str, oauth_provider_ids: Dict[str, str],
                               software_background: Optional[List[str]] = None,
                               hardware_background: Optional[List[str]] = None) -> User:
        if self.get_user_by_email(email):
            raise UserAlreadyExistsException()

        user = User(email=email, oauth_provider_ids=oauth_provider_ids)
        self.session.add(user)
        self.session.commit()
        self.session.refresh(user)

        if software_background or hardware_background:
            personalization_data = PersonalizationData(
                user_id=user.id,
                software_background=software_background or [],
                hardware_background=hardware_background or []
            )
            self.session.add(personalization_data)
            self.session.commit()
            self.session.refresh(personalization_data)
            user.personalization_data = personalization_data # Link personalization data to user

        return user
