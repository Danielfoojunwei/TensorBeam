"""Main API Application."""

from fastapi import FastAPI, Depends, HTTPException, BackgroundTasks
from sqlalchemy.orm import Session
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from platform.backend.internal.config import settings
from platform.backend.internal.models import Base, Tenant, Job, EvalKey, ModelPack
from platform.backend.internal.inference.engine import MockInferenceEngine

app = FastAPI(title=settings.APP_NAME)

# DB Setup
engine = create_engine(settings.DATABASE_URL, connect_args={"check_same_thread": False})
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base.metadata.create_all(bind=engine)

# Inference Setup
inference_engine = MockInferenceEngine()

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# --- Endpoints ---

@app.get("/healthz")
def healthz():
    return {"status": "ok", "backend": settings.BACKEND_TYPE}

@app.post("/tenants")
def create_tenant(name: str, db: Session = Depends(get_db)):
    db_tenant = Tenant(name=name)
    db.add(db_tenant)
    db.commit()
    return {"id": db_tenant.id, "name": db_tenant.name}

@app.post("/tenants/{tenant_id}/eval-keys")
def upload_keys(tenant_id: str, db: Session = Depends(get_db)):
    # Mocking file upload handling
    # In real impl, stream bytes to Storage
    key_path = f"keys/{tenant_id}/v1.evk"
    
    db_key = EvalKey(tenant_id=tenant_id, version="v1", storage_path=key_path)
    db.add(db_key)
    db.commit()
    
    # Notify engine
    inference_engine.register_keys(tenant_id, key_path)
    
    return {"status": "registered", "key_id": db_key.id}

async def process_job(job_id: str, tenant_id: str, model_id: str, input_path: str):
    """Async worker function."""
    db = SessionLocal()
    job = db.query(Job).get(job_id)
    try:
        job.status = "PROCESSING"
        job.started_at = datetime.utcnow()
        db.commit()
        
        # Load input (mock)
        input_data = b"MOCK_INPUT_CIPHERTEXT" 
        
        # Run inference
        output = inference_engine.submit_batch(tenant_id, model_id, input_data)
        
        # Save output (mock)
        output_path = f"outputs/{job_id}.result"
        
        job.status = "COMPLETED"
        job.completed_at = datetime.utcnow()
        job.output_path = output_path
        
    except Exception as e:
        job.status = "FAILED"
        job.error_message = str(e)
    finally:
        db.commit()
        db.close()

@app.post("/jobs")
def submit_job(tenant_id: str, model_id: str, background_tasks: BackgroundTasks, db: Session = Depends(get_db)):
    """Submit async inference job."""
    # Validate tenant/model
    
    job = Job(tenant_id=tenant_id, model_id=model_id, input_path="inputs/req.ctx")
    db.add(job)
    db.commit()
    
    background_tasks.add_task(process_job, job.id, tenant_id, model_id, job.input_path)
    
    return {"job_id": job.id, "status": "QUEUED"}

@app.get("/jobs/{job_id}")
def get_job(job_id: str, db: Session = Depends(get_db)):
    job = db.query(Job).get(job_id)
    if not job:
        raise HTTPException(status_code=404, detail="Job not found")
    return {
        "id": job.id,
        "status": job.status,
        "result_uri": job.output_path if job.status == "COMPLETED" else None,
        "error": job.error_message
    }
