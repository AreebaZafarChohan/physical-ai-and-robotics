from fastapi import HTTPException, status

class CredentialException(HTTPException):
    def __init__(self, detail: str = "Could not validate credentials"):
        super().__init__(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=detail,
            headers={"WWW-Authenticate": "Bearer"},
        )

class UserAlreadyExistsException(Exception):
    def __init__(self, detail: str = "User with this email already exists"):
        self.detail = detail
        super().__init__(self.detail)

class InvalidCredentialsException(HTTPException):
    def __init__(self, detail: str = "Incorrect email or password"):
        super().__init__(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=detail,
            headers={"WWW-Authenticate": "Bearer"},
        )
