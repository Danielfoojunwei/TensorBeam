"""FHE Client SDK for confidential inference.

This SDK handles:
1. Key generation (secret key stays local)
2. Input encryption
3. Registering evaluation keys with server
4. Calling inference endpoint
5. Decrypting results

The secret key NEVER leaves the client.

Usage:
    client = FHEClient(server_url="http://localhost:8000")

    # Generate keys
    client.setup()

    # Register eval keys with server
    client.register_keys()

    # Encrypt and infer
    embedding = np.random.randn(64).astype(np.float64)
    result = client.infer("demo-linear-classifier", embedding)

    # Result contains decrypted logits
    print(result.logits)
    print(result.predicted_class)
"""

from __future__ import annotations

import base64
import logging
import uuid
from dataclasses import dataclass
from typing import Any

import httpx
import numpy as np
import numpy.typing as npt

from fhe_inference_service.core.backend import FHEScheme, KeyBundle, ParamsConfig
from fhe_inference_service.core.backends.tenseal_ckks import TenSEALCKKSBackend

logger = logging.getLogger(__name__)


@dataclass
class InferenceResult:
    """Result of encrypted inference.

    Attributes:
        logits: Decrypted output logits.
        predicted_class: Index of highest logit (argmax).
        probabilities: Softmax probabilities (if computed).
        metadata: Server-side metadata (latency, sizes).
    """

    logits: npt.NDArray[np.floating[Any]]
    predicted_class: int
    probabilities: npt.NDArray[np.floating[Any]] | None = None
    metadata: dict[str, Any] | None = None


def softmax(x: npt.NDArray[np.floating[Any]]) -> npt.NDArray[np.floating[Any]]:
    """Compute softmax probabilities."""
    exp_x = np.exp(x - np.max(x))  # Numerical stability
    return exp_x / exp_x.sum()


class FHEClient:
    """Client SDK for FHE Inference Service.

    Handles key generation, encryption, API calls, and decryption.
    The secret key NEVER leaves this client.

    Attributes:
        server_url: Base URL of the FHE inference server.
        session_id: Unique session identifier.
        backend: FHE backend implementation.
        params_config: FHE parameters configuration.
    """

    def __init__(
        self,
        server_url: str = "http://localhost:8000",
        params_config: ParamsConfig | None = None,
        timeout: float = 300.0,  # 5 minutes for FHE operations
    ) -> None:
        """Initialize FHE client.

        Args:
            server_url: Base URL of the inference server.
            params_config: FHE parameters (uses defaults if None).
            timeout: HTTP request timeout in seconds.
        """
        self.server_url = server_url.rstrip("/")
        self.session_id = str(uuid.uuid4())
        self.backend = TenSEALCKKSBackend()
        self.params_config = params_config or ParamsConfig()
        self.timeout = timeout

        # Keys (populated by setup())
        self._key_bundle: KeyBundle | None = None
        self._context: bytes | None = None

        # HTTP client
        self._http = httpx.Client(timeout=timeout)

        logger.info(f"FHE Client initialized: session={self.session_id}")

    def setup(self) -> None:
        """Set up FHE context and generate keys.

        This generates:
        - Secret key (kept locally, NEVER sent to server)
        - Public key
        - Evaluation keys (sent to server for encrypted operations)

        Call this before any other operations.
        """
        logger.info("Setting up FHE context and generating keys...")

        # Create context with keys
        self._context = self.backend.setup_context(self.params_config)

        # Generate/extract keys
        self._key_bundle = self.backend.keygen(self._context)

        logger.info(
            f"Keys generated: "
            f"secret_key={len(self._key_bundle.secret_key)} bytes, "
            f"eval_keys={len(self._key_bundle.eval_keys)} bytes"
        )

    def register_keys(self) -> dict[str, Any]:
        """Register evaluation keys with the server.

        Sends only the PUBLIC evaluation keys (galois + relinearization).
        The secret key stays local.

        Returns:
            Server response confirming registration.

        Raises:
            RuntimeError: If setup() hasn't been called.
            httpx.HTTPError: If server request fails.
        """
        if self._key_bundle is None:
            raise RuntimeError("Must call setup() before register_keys()")

        logger.info(f"Registering eval keys with server: {self.server_url}")

        # Encode keys as base64
        context_b64 = base64.b64encode(self._key_bundle.public_key).decode("ascii")
        eval_keys_b64 = base64.b64encode(self._key_bundle.eval_keys).decode("ascii")

        response = self._http.post(
            f"{self.server_url}/v1/keys/register",
            json={
                "session_id": self.session_id,
                "fhe_scheme": self.params_config.scheme.value,
                "params": {
                    "poly_modulus_degree": self.params_config.poly_modulus_degree,
                    "security_level": self.params_config.security_level,
                },
                "context_b64": context_b64,
                "eval_keys_b64": eval_keys_b64,
            },
        )
        response.raise_for_status()

        result = response.json()
        logger.info(f"Keys registered: {result}")
        return result

    def encrypt(
        self, plaintext: npt.NDArray[np.floating[Any]]
    ) -> bytes:
        """Encrypt a plaintext vector.

        Args:
            plaintext: Input vector to encrypt.

        Returns:
            Serialized ciphertext bytes.

        Raises:
            RuntimeError: If setup() hasn't been called.
        """
        if self._key_bundle is None or self._context is None:
            raise RuntimeError("Must call setup() before encrypt()")

        return self.backend.encrypt(
            context=self._context,
            secret_key=self._key_bundle.secret_key,
            plaintext=plaintext,
        )

    def decrypt(
        self, ciphertext: bytes, output_size: int
    ) -> npt.NDArray[np.floating[Any]]:
        """Decrypt a ciphertext to plaintext.

        Args:
            ciphertext: Serialized ciphertext.
            output_size: Expected output size.

        Returns:
            Decrypted plaintext vector.

        Raises:
            RuntimeError: If setup() hasn't been called.
        """
        if self._key_bundle is None or self._context is None:
            raise RuntimeError("Must call setup() before decrypt()")

        return self.backend.decrypt(
            context=self._context,
            secret_key=self._key_bundle.secret_key,
            ciphertext=ciphertext,
            output_size=output_size,
        )

    def infer(
        self,
        model_id: str,
        embedding: npt.NDArray[np.floating[Any]],
        apply_softmax: bool = True,
    ) -> InferenceResult:
        """Run encrypted inference on an embedding.

        Full pipeline:
        1. Encrypt embedding locally
        2. Send ciphertext to server
        3. Server runs encrypted inference
        4. Receive encrypted output
        5. Decrypt locally
        6. Optionally apply softmax

        Args:
            model_id: Model to use for inference.
            embedding: Input embedding vector.
            apply_softmax: Whether to apply softmax to logits.

        Returns:
            InferenceResult with decrypted logits and predicted class.

        Raises:
            RuntimeError: If setup() or register_keys() haven't been called.
            httpx.HTTPError: If server request fails.
        """
        if self._key_bundle is None:
            raise RuntimeError("Must call setup() and register_keys() before infer()")

        logger.info(f"Running encrypted inference: model={model_id}")

        # Encrypt input
        ciphertext = self.encrypt(embedding.astype(np.float64))
        ct_b64 = base64.b64encode(ciphertext).decode("ascii")

        # Call inference endpoint
        response = self._http.post(
            f"{self.server_url}/v1/infer",
            json={
                "session_id": self.session_id,
                "model_id": model_id,
                "ciphertext_input_b64": ct_b64,
            },
        )
        response.raise_for_status()

        result = response.json()
        metadata = result.get("metadata", {})

        # Decode and decrypt output
        ct_output = base64.b64decode(result["ciphertext_output_b64"])

        # Get output size from model metadata
        output_size = self._get_model_output_size(model_id)
        logits = self.decrypt(ct_output, output_size)

        # Compute predicted class and optionally softmax
        predicted_class = int(np.argmax(logits))
        probabilities = softmax(logits) if apply_softmax else None

        logger.info(
            f"Inference complete: predicted_class={predicted_class}, "
            f"latency={metadata.get('latency_ms', '?')}ms"
        )

        return InferenceResult(
            logits=logits,
            predicted_class=predicted_class,
            probabilities=probabilities,
            metadata=metadata,
        )

    def _get_model_output_size(self, model_id: str) -> int:
        """Get output size for a model from server metadata."""
        response = self._http.get(f"{self.server_url}/v1/models")
        response.raise_for_status()

        models = response.json().get("models", [])
        for model in models:
            if model["model_id"] == model_id:
                return model["output_dim"]

        # Default fallback
        logger.warning(f"Model {model_id} not found in registry, using default output_size=3")
        return 3

    def list_models(self) -> list[dict[str, Any]]:
        """List available models on the server."""
        response = self._http.get(f"{self.server_url}/v1/models")
        response.raise_for_status()
        return response.json().get("models", [])

    def health_check(self) -> dict[str, Any]:
        """Check server health."""
        response = self._http.get(f"{self.server_url}/v1/health")
        response.raise_for_status()
        return response.json()

    def close(self) -> None:
        """Close HTTP client."""
        self._http.close()

    def __enter__(self) -> "FHEClient":
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()
