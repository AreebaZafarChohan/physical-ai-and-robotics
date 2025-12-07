from fastapi import FastAPI  # type: ignore
from fastapi.middleware.cors import CORSMiddleware # type: ignore
from backend.app.api import embed, query, ask, selected_text, log
from backend.app.utils.error_handlers import add_exception_handlers # Added import

app = FastAPI()

origins = [
    "http://localhost",
    "http://localhost:3000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

add_exception_handlers(app) # Added this line

app.include_router(embed.router, prefix="/api/v1", tags=["Embed"])
app.include_router(query.router, prefix="/api/v1", tags=["Query"])
app.include_router(ask.router, prefix="/api/v1", tags=["Ask"])
app.include_router(selected_text.router, prefix="/api/v1", tags=["Selected Text"])
app.include_router(log.router, prefix="/api/v1", tags=["Log"])

@app.get("/")
async def root():
    return {"message": "FastAPI Backend is running!"}
 # type: ignore