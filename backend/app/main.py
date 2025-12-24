from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
load_dotenv()

app = FastAPI(
    title="Unified Backend API",
    description="AI Chat + Urdu Translation",
    version="1.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000","https://paihr.vercel.app/"], 
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


from app.agent import router as agent_router
from src.translate import router as translation_router

app.include_router(agent_router, prefix="/agent")
app.include_router(translation_router, prefix="/api") 

@app.get("/")
def root():
    return {"message": "Unified Backend API running"}
