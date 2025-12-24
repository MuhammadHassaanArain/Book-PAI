from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

app = FastAPI(
    title="Urdu Translation API",
    description="API for translating book chapter content to Urdu",
    version="1.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "*",
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://localhost:5173",
        "http://127.0.0.1:5173",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

from backend.src.api.translation import router as translation_router
app.include_router(translation_router, prefix="/api", tags=["translation"])

@app.get("/")
def read_root():
    return {"message": "Urdu Translation API is running"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "backend.src.main:app",
        host=os.getenv("BACKEND_HOST", "0.0.0.0"),
        port=int(os.getenv("BACKEND_PORT", 8000)),
        reload=True
    )
