"""Database Models (SQLAlchemy)."""

from sqlalchemy import Column, String, Integer, DateTime, ForeignKey, JSON
from sqlalchemy.orm import declarative_base, relationship
from datetime import datetime
import uuid

Base = declarative_base()

def generate_id():
    return str(uuid.uuid4())

class Tenant(Base):
    __tablename__ = "tenants"
    
    id = Column(String, primary_key=True, default=generate_id)
    name = Column(String, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    
    keys = relationship("EvalKey", back_populates="tenant")
    models = relationship("ModelPack", back_populates="tenant")
    jobs = relationship("Job", back_populates="tenant")

class EvalKey(Base):
    __tablename__ = "eval_keys"
    
    id = Column(String, primary_key=True, default=generate_id)
    tenant_id = Column(String, ForeignKey("tenants.id"))
    version = Column(String, nullable=False)
    storage_path = Column(String, nullable=False) # Path to encrypted key file
    created_at = Column(DateTime, default=datetime.utcnow)
    expires_at = Column(DateTime, nullable=True)
    
    tenant = relationship("Tenant", back_populates="keys")

class ModelPack(Base):
    __tablename__ = "model_packs"
    
    id = Column(String, primary_key=True, default=generate_id)
    tenant_id = Column(String, ForeignKey("tenants.id"))
    name = Column(String, nullable=False)
    version = Column(String, nullable=False)
    storage_path = Column(String, nullable=False)
    status = Column(String, default="DEV") # DEV, STAGING, PROD
    
    tenant = relationship("Tenant", back_populates="models")

class Job(Base):
    __tablename__ = "jobs"
    
    id = Column(String, primary_key=True, default=generate_id)
    tenant_id = Column(String, ForeignKey("tenants.id"))
    model_id = Column(String, ForeignKey("model_packs.id"))
    status = Column(String, default="QUEUED") # QUEUED, PROCESSING, COMPLETED, FAILED
    
    input_path = Column(String)
    output_path = Column(String, nullable=True)
    
    created_at = Column(DateTime, default=datetime.utcnow)
    started_at = Column(DateTime, nullable=True)
    completed_at = Column(DateTime, nullable=True)
    
    error_message = Column(String, nullable=True)
    metrics = Column(JSON, nullable=True)
    
    tenant = relationship("Tenant", back_populates="jobs")
