"""MOAI Platform Backend Configuration."""

from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    # App
    APP_NAME: str = "MOAI Control Plane"
    ENV: str = "dev"
    
    # Database
    DATABASE_URL: str = "sqlite:///./moai_platform.db"
    
    # Security
    SECRET_KEY: str = "insecure-dev-key-change-me"
    ALGORITHM: str = "HS256"
    
    # Inference
    BACKEND_TYPE: str = "mock"  # mock or native
    LIBMOAI_PATH: str = "/usr/lib/libmoai_gpu.so"
    
    # Storage
    STORAGE_TYPE: str = "local" # local or s3
    STORAGE_PATH: str = "./storage"

    class Config:
        env_file = ".env"

settings = Settings()
