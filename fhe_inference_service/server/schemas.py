"""Pydantic schemas for API request/response models."""

from __future__ import annotations

from pydantic import BaseModel, Field


class KeysRegisterRequest(BaseModel):
    """Request to register evaluation keys for a session.

    The client generates keys locally and sends only the evaluation keys
    (galois + relinearization) to the server. The secret key NEVER leaves the client.
    """

    session_id: str = Field(..., description="Unique session identifier (UUID)")
    fhe_scheme: str = Field(default="ckks", description="FHE scheme (ckks, bfv, tfhe)")
    params: dict[str, int | float | list[int]] = Field(
        default_factory=dict,
        description="FHE parameters (poly_modulus_degree, etc.)",
    )
    context_b64: str = Field(..., description="Base64-encoded public context")
    eval_keys_b64: str = Field(..., description="Base64-encoded evaluation keys")


class KeysRegisterResponse(BaseModel):
    """Response after registering keys."""

    session_id: str
    status: str = "registered"
    message: str = "Evaluation keys registered successfully"


class InferRequest(BaseModel):
    """Request to run encrypted inference."""

    session_id: str = Field(..., description="Session ID with registered keys")
    model_id: str = Field(..., description="Model to use for inference")
    ciphertext_input_b64: str = Field(..., description="Base64-encoded ciphertext")


class InferResponse(BaseModel):
    """Response with encrypted inference result."""

    ciphertext_output_b64: str = Field(..., description="Base64-encoded output ciphertext")
    metadata: InferMetadata


class InferMetadata(BaseModel):
    """Metadata about the inference operation."""

    latency_ms: float = Field(..., description="Server-side inference latency in ms")
    ct_input_bytes: int = Field(..., description="Input ciphertext size in bytes")
    ct_output_bytes: int = Field(..., description="Output ciphertext size in bytes")
    model_id: str = Field(..., description="Model used for inference")


class ModelInfo(BaseModel):
    """Information about a registered model."""

    model_id: str
    input_dim: int
    output_dim: int
    scheme: str
    description: str
    model_type: str


class ModelsListResponse(BaseModel):
    """Response listing available models."""

    models: list[ModelInfo]


class HealthResponse(BaseModel):
    """Health check response."""

    status: str = "healthy"
    version: str
    backend: str


class ErrorResponse(BaseModel):
    """Error response."""

    error: str
    detail: str | None = None
