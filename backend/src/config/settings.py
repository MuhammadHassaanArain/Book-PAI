from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    gemini_api_key: str
    backend_host: str = "0.0.0.0"
    backend_port: int = 8000
    debug: bool = False
    frontend_url: str = "http://localhost:3000"

    class Config:
        env_file = ".env"

settings = Settings()