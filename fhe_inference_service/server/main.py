"""FastAPI server for FHE Inference Service.

MOAI-style Confidential Cognition Service:
- Receives ciphertext payloads + evaluation keys
- Runs encrypted inference (linear/MLP classifiers)
- Returns ciphertext outputs
- NEVER sees secret keys or plaintext data

Use cases (slow-loop cognition):
- Compliance/risk gates
- SOP reranker
- Sensitive incident classification
- Approvals/constraints checking

API Version: v1
"""

from __future__ import annotations

import base64
import logging
import time
from contextlib import asynccontextmanager
from typing import Any

from fastapi import FastAPI, HTTPException, status

from fhe_inference_service import __version__
from fhe_inference_service.core.backends.tenseal_ckks import TenSEALCKKSBackend
from fhe_inference_service.core.model_registry import (
    get_global_registry,
    register_default_models,
)
from fhe_inference_service.server.schemas import (
    ErrorResponse,
    HealthResponse,
    InferMetadata,
    InferRequest,
    InferResponse,
    KeysRegisterRequest,
    KeysRegisterResponse,
    ModelInfo,
    ModelsListResponse,
)
from fhe_inference_service.server.session_store import SessionStore

# Logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global instances
session_store = SessionStore(default_ttl_seconds=3600)
backend = TenSEALCKKSBackend()


@asynccontextmanager
async def lifespan(app: FastAPI):  # noqa: ARG001
    """Application lifespan handler."""
    # Startup
    logger.info("Starting FHE Inference Service...")
    register_default_models()
    logger.info(f"Loaded {len(get_global_registry().list_models())} models")
    yield
    # Shutdown
    logger.info("Shutting down FHE Inference Service...")


app = FastAPI(
    title="FHE Inference Service",
    description="MOAI-style Confidential Cognition - Encrypted inference on sensitive data",
    version=__version__,
    lifespan=lifespan,
    responses={
        400: {"model": ErrorResponse},
        404: {"model": ErrorResponse},
        500: {"model": ErrorResponse},
    },
)


# =============================================================================
# Health Check
# =============================================================================


@app.get("/v1/health", response_model=HealthResponse, tags=["Health"])
async def health_check() -> HealthResponse:
    """Health check endpoint.

    Returns service status, version, and active backend.
    """
    return HealthResponse(
        status="healthy",
        version=__version__,
        backend=backend.name,
    )


# =============================================================================
# Model Management
# =============================================================================


@app.get("/v1/models", response_model=ModelsListResponse, tags=["Models"])
async def list_models() -> ModelsListResponse:
    """List available inference models.

    Returns all registered models with their metadata
    (input/output dimensions, required FHE scheme).
    """
    registry = get_global_registry()
    models = [
        ModelInfo(
            model_id=m.model_id,
            input_dim=m.input_dim,
            output_dim=m.output_dim,
            scheme=m.scheme.value,
            description=m.description,
            model_type=m.model_type,
        )
        for m in registry.list_models()
    ]
    return ModelsListResponse(models=models)


# =============================================================================
# Key Management
# =============================================================================


@app.post(
    "/v1/keys/register",
    response_model=KeysRegisterResponse,
    status_code=status.HTTP_201_CREATED,
    tags=["Keys"],
)
async def register_keys(request: KeysRegisterRequest) -> KeysRegisterResponse:
    """Register evaluation keys for a session.

    The client generates keys locally and sends only the PUBLIC evaluation keys
    (galois + relinearization) to the server. The SECRET KEY must NEVER be sent.

    Keys are stored ephemerally and expire after 1 hour.

    Args:
        request: Session ID, FHE scheme, context, and eval keys (base64).

    Returns:
        Confirmation of key registration.
    """
    try:
        # Decode base64 payloads
        context = base64.b64decode(request.context_b64)
        eval_keys = base64.b64decode(request.eval_keys_b64)

        # Validate scheme
        if request.fhe_scheme.lower() not in ("ckks", "bfv", "tfhe"):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Unsupported FHE scheme: {request.fhe_scheme}",
            )

        # Store session
        session_store.register(
            session_id=request.session_id,
            fhe_scheme=request.fhe_scheme.lower(),
            context=context,
            eval_keys=eval_keys,
            params=dict(request.params),
        )

        logger.info(
            f"Registered keys for session {request.session_id} "
            f"(scheme: {request.fhe_scheme}, context: {len(context)} bytes)"
        )

        return KeysRegisterResponse(
            session_id=request.session_id,
            status="registered",
            message="Evaluation keys registered successfully",
        )

    except base64.binascii.Error as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid base64 encoding: {e}",
        ) from e


# =============================================================================
# Inference
# =============================================================================


@app.post("/v1/infer", response_model=InferResponse, tags=["Inference"])
async def run_inference(request: InferRequest) -> InferResponse:
    """Run encrypted inference.

    Takes a ciphertext input, runs the specified model on encrypted data,
    and returns the encrypted output. The server NEVER sees plaintext.

    The client must:
    1. Register evaluation keys first (POST /v1/keys/register)
    2. Encrypt input locally with their secret key
    3. Decrypt output locally with their secret key

    Args:
        request: Session ID, model ID, and encrypted input (base64).

    Returns:
        Encrypted inference output with metadata.
    """
    # Validate session
    session = session_store.get(request.session_id)
    if session is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Session not found or expired: {request.session_id}",
        )

    # Validate model
    registry = get_global_registry()
    model = registry.get(request.model_id)
    if model is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Model not found: {request.model_id}",
        )

    try:
        # Decode input ciphertext
        ct_input = base64.b64decode(request.ciphertext_input_b64)
        ct_input_size = len(ct_input)

        # Run encrypted inference
        start_time = time.perf_counter()

        ct_output = backend.eval(
            context=session.context,
            eval_keys=session.eval_keys,
            ciphertext=ct_input,
            model=model,
        )

        elapsed_ms = (time.perf_counter() - start_time) * 1000
        ct_output_size = len(ct_output)

        logger.info(
            f"Inference complete: session={request.session_id}, "
            f"model={request.model_id}, latency={elapsed_ms:.2f}ms"
        )

        # Encode output
        ct_output_b64 = base64.b64encode(ct_output).decode("ascii")

        return InferResponse(
            ciphertext_output_b64=ct_output_b64,
            metadata=InferMetadata(
                latency_ms=round(elapsed_ms, 2),
                ct_input_bytes=ct_input_size,
                ct_output_bytes=ct_output_size,
                model_id=request.model_id,
            ),
        )

    except base64.binascii.Error as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid base64 encoding: {e}",
        ) from e
    except Exception as e:
        logger.exception(f"Inference error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Inference failed: {e}",
        ) from e


# =============================================================================
# Session Management (optional endpoints)
# =============================================================================


@app.delete("/v1/keys/{session_id}", tags=["Keys"])
async def delete_session(session_id: str) -> dict[str, Any]:
    """Delete a session and its evaluation keys.

    Args:
        session_id: Session to delete.

    Returns:
        Confirmation of deletion.
    """
    if session_store.delete(session_id):
        return {"status": "deleted", "session_id": session_id}
    raise HTTPException(
        status_code=status.HTTP_404_NOT_FOUND,
        detail=f"Session not found: {session_id}",
    )


# =============================================================================
# Future: Batch endpoint (v2)
# =============================================================================
# @app.post("/v2/infer/batch")
# async def run_batch_inference(...):
#     """Batch inference with micro-batching and SLA tiers.
#
#     Supports:
#     - Micro-batching (queue for T_batch_window)
#     - Batch tier (min batch size for cost efficiency)
#     - Priority tier (smaller batches, higher cost)
#     """
#     pass
