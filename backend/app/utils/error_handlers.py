from fastapi import FastAPI, Request, HTTPException, status
from fastapi.responses import JSONResponse
from openai import OpenAIError
from qdrant_client.http.exceptions import UnexpectedResponse

# Custom exception for external service errors
class ExternalServiceError(HTTPException):
    def __init__(self, service_name: str, original_detail: str, status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR):
        super().__init__(status_code=status_code, detail=f"Error with {service_name} service: {original_detail}")
        self.service_name = service_name
        self.original_detail = original_detail

def add_exception_handlers(app: FastAPI):
    @app.exception_handler(OpenAIError)
    async def openai_exception_handler(request: Request, exc: OpenAIError):
        return JSONResponse(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            content={"detail": f"OpenAI service unavailable or returned an error: {exc.json_body['message'] if hasattr(exc, 'json_body') else str(exc)}"},
        )

    @app.exception_handler(UnexpectedResponse)
    async def qdrant_exception_handler(request: Request, exc: UnexpectedResponse):
        return JSONResponse(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            content={"detail": f"Qdrant service unavailable or returned an error: {exc.reason}"},
        )

    @app.exception_handler(ExternalServiceError)
    async def external_service_exception_handler(request: Request, exc: ExternalServiceError):
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.detail},
        )

    @app.exception_handler(ValueError)
    async def value_error_handler(request: Request, exc: ValueError):
        return JSONResponse(
            status_code=status.HTTP_400_BAD_REQUEST,
            content={"detail": str(exc)},
        )

    # Generic HTTPException handler (already covered by FastAPI, but can be customized)
    @app.exception_handler(HTTPException)
    async def http_exception_handler(request: Request, exc: HTTPException):
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.detail},
            headers=exc.headers,
        )
