"""MOAI Client SDK.

Provides client-side encryption, job submission, and decryption.
The secret key NEVER leaves this client.
"""

from __future__ import annotations

import logging
import time
import uuid
from dataclasses import dataclass
from typing import Any

import numpy as np
import numpy.typing as npt

logger = logging.getLogger(__name__)


@dataclass
class InferenceResult:
    """Result from encrypted inference."""

    logits: npt.NDArray[np.floating[Any]]
    metadata: dict[str, Any] | None = None
    probabilities: npt.NDArray[np.floating[Any]] | None = None
    predicted_class: int | None = None


def softmax(x: npt.NDArray[np.floating[Any]]) -> npt.NDArray[np.floating[Any]]:
    """Compute softmax probabilities."""
    exp_x = np.exp(x - np.max(x))
    return exp_x / np.sum(exp_x)


class MOAIClient:
    """MOAI FHE Client SDK.

    Handles:
    - Key generation (secret key stays local)
    - Input encryption
    - Job submission
    - Result decryption

    This client can work with either:
    - REST API (using httpx)
    - gRPC (using grpcio)
    """

    def __init__(
        self,
        server_url: str = "http://localhost:8000",
        use_grpc: bool = False,
    ) -> None:
        """Initialize client.

        Args:
            server_url: MOAI service URL.
            use_grpc: Use gRPC instead of REST.
        """
        self.server_url = server_url.rstrip("/")
        self.use_grpc = use_grpc
        self.session_id = str(uuid.uuid4())

        # FHE state (populated by setup())
        self._context: bytes | None = None
        self._secret_key: bytes | None = None
        self._eval_keys: bytes | None = None
        self._backend: Any | None = None

        # HTTP client (lazy init)
        self._http_client: Any | None = None

    def setup(self, poly_modulus_degree: int = 8192) -> None:
        """Generate FHE keys locally.

        The secret key NEVER leaves this client.
        """
        # Import here to avoid circular deps and allow optional install
        from moai_core.backend import ParamsConfig
        from moai_core.backends.tenseal_ckks import TenSEALCKKSBackend

        self._backend = TenSEALCKKSBackend()
        config = ParamsConfig(poly_modulus_degree=poly_modulus_degree)

        # Generate context and keys
        self._context = self._backend.setup_context(config)
        key_bundle = self._backend.keygen(self._context)

        # SECRET KEY STAYS LOCAL
        self._secret_key = key_bundle.secret_key
        self._eval_keys = key_bundle.eval_keys

        logger.info("FHE keys generated locally")
        logger.warning("Secret key NEVER leaves this client")

    def register_keys(self) -> dict[str, Any]:
        """Register evaluation keys with server.

        Only sends public context and eval keys. Secret key stays local.
        """
        if self._eval_keys is None:
            raise RuntimeError("Must call setup() first")

        import base64
        import httpx

        payload = {
            "session_id": self.session_id,
            "fhe_scheme": "ckks",
            "params": {},
            "context_b64": base64.b64encode(self._eval_keys).decode("ascii"),
            "eval_keys_b64": base64.b64encode(self._eval_keys).decode("ascii"),
        }

        with httpx.Client(timeout=60.0) as client:
            response = client.post(
                f"{self.server_url}/v1/keys/register",
                json=payload,
            )
            response.raise_for_status()

        logger.info(f"Eval keys registered, session: {self.session_id}")
        return response.json()

    def encrypt(
        self, embedding: npt.NDArray[np.floating[Any]]
    ) -> bytes:
        """Encrypt an embedding vector.

        Args:
            embedding: Input embedding (plaintext).

        Returns:
            Encrypted ciphertext bytes.
        """
        if self._backend is None or self._context is None:
            raise RuntimeError("Must call setup() first")

        return self._backend.encrypt(self._context, self._secret_key, embedding)

    def decrypt(self, ciphertext: bytes, output_size: int) -> npt.NDArray[np.floating[Any]]:
        """Decrypt ciphertext to plaintext.

        Args:
            ciphertext: Encrypted output.
            output_size: Expected output dimension.

        Returns:
            Decrypted vector.
        """
        if self._backend is None or self._context is None:
            raise RuntimeError("Must call setup() first")

        return self._backend.decrypt(
            self._context, self._secret_key, ciphertext, output_size
        )

    def infer(
        self,
        model_id: str,
        embedding: npt.NDArray[np.floating[Any]],
        output_dim: int = 3,
    ) -> InferenceResult:
        """Run encrypted inference.

        Complete flow:
        1. Encrypt embedding locally
        2. Submit to server
        3. Decrypt result locally
        4. Return logits and predictions

        Args:
            model_id: Model to use.
            embedding: Input embedding.
            output_dim: Expected output dimension.

        Returns:
            InferenceResult with decrypted logits.
        """
        import base64
        import httpx

        # Encrypt locally
        ciphertext = self.encrypt(embedding)

        # Submit to server
        payload = {
            "session_id": self.session_id,
            "model_id": model_id,
            "ciphertext_input_b64": base64.b64encode(ciphertext).decode("ascii"),
        }

        with httpx.Client(timeout=120.0) as client:
            response = client.post(
                f"{self.server_url}/v1/infer",
                json=payload,
            )
            response.raise_for_status()

        data = response.json()
        ct_output = base64.b64decode(data["ciphertext_output_b64"])

        # Decrypt locally
        logits = self.decrypt(ct_output, output_dim)

        # Compute probabilities
        probs = softmax(logits)
        predicted = int(np.argmax(probs))

        return InferenceResult(
            logits=logits,
            probabilities=probs,
            predicted_class=predicted,
            metadata=data.get("metadata"),
        )

    def list_models(self) -> list[dict[str, Any]]:
        """List available models."""
        import httpx

        with httpx.Client(timeout=30.0) as client:
            response = client.get(f"{self.server_url}/v1/models")
            response.raise_for_status()

        return response.json().get("models", [])

    def health_check(self) -> dict[str, Any]:
        """Check server health."""
        import httpx

        with httpx.Client(timeout=10.0) as client:
            response = client.get(f"{self.server_url}/v1/health")
            response.raise_for_status()

        return response.json()

    def close(self) -> None:
        """Clean up resources."""
        pass

    def __enter__(self) -> "MOAIClient":
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()
