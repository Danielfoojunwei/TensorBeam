"""Model registry for FHE-compatible inference models.

Manages model registration, lookup, and metadata for encrypted inference.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from fhe_inference_service.core.backend import FHEScheme

if TYPE_CHECKING:
    from fhe_inference_service.core.backend import InferenceModel

logger = logging.getLogger(__name__)


@dataclass
class ModelMetadata:
    """Metadata about a registered model.

    Attributes:
        model_id: Unique identifier.
        input_dim: Expected input dimension.
        output_dim: Output dimension.
        scheme: Required FHE scheme.
        description: Human-readable description.
        model_type: Type of model (linear, mlp, etc.).
    """

    model_id: str
    input_dim: int
    output_dim: int
    scheme: FHEScheme
    description: str = ""
    model_type: str = "linear"


@dataclass
class ModelRegistry:
    """Registry for FHE inference models.

    Thread-safe model storage and retrieval.
    Future: Load models from disk, support dynamic registration.
    """

    _models: dict[str, "InferenceModel"] = field(default_factory=dict)
    _metadata: dict[str, ModelMetadata] = field(default_factory=dict)

    def register(
        self,
        model: "InferenceModel",
        description: str = "",
        model_type: str = "linear",
        scheme: FHEScheme = FHEScheme.CKKS,
    ) -> None:
        """Register a model in the registry.

        Args:
            model: The model to register.
            description: Human-readable description.
            model_type: Type of model.
            scheme: Required FHE scheme.
        """
        model_id = model.model_id
        self._models[model_id] = model
        self._metadata[model_id] = ModelMetadata(
            model_id=model_id,
            input_dim=model.input_dim,
            output_dim=model.output_dim,
            scheme=scheme,
            description=description,
            model_type=model_type,
        )
        logger.info(f"Registered model: {model_id} ({model_type}, {scheme.value})")

    def get(self, model_id: str) -> "InferenceModel | None":
        """Get a model by ID."""
        return self._models.get(model_id)

    def get_metadata(self, model_id: str) -> ModelMetadata | None:
        """Get model metadata by ID."""
        return self._metadata.get(model_id)

    def list_models(self) -> list[ModelMetadata]:
        """List all registered models."""
        return list(self._metadata.values())

    def __contains__(self, model_id: str) -> bool:
        return model_id in self._models


# Global registry instance
_global_registry: ModelRegistry | None = None


def get_global_registry() -> ModelRegistry:
    """Get the global model registry (singleton)."""
    global _global_registry
    if _global_registry is None:
        _global_registry = ModelRegistry()
    return _global_registry


def register_default_models() -> None:
    """Register default demo models."""
    from fhe_inference_service.core.models.linear_model import (
        create_demo_linear_classifier,
        create_demo_mlp_classifier,
    )

    registry = get_global_registry()

    # Register demo linear classifier
    linear_model = create_demo_linear_classifier()
    registry.register(
        linear_model,
        description="Demo linear classifier (3 classes, 64-dim embeddings)",
        model_type="linear",
    )

    # Register demo MLP classifier
    mlp_model = create_demo_mlp_classifier()
    registry.register(
        mlp_model,
        description="Demo 2-layer MLP with polynomial activation (3 classes)",
        model_type="mlp",
    )

    logger.info(f"Registered {len(registry.list_models())} default models")
