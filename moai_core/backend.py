"""Abstract FHE Backend interface.

This module defines the abstract interface for FHE backends.
Swap implementations to use different FHE libraries (TenSEAL, Concrete, MOAI kernels).
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Any, Protocol

import numpy as np
import numpy.typing as npt


class FHEScheme(str, Enum):
    """Supported FHE schemes."""

    CKKS = "ckks"  # Approximate arithmetic (real numbers)
    BFV = "bfv"  # Exact arithmetic (integers)
    TFHE = "tfhe"  # Boolean/integer circuits


@dataclass
class ParamsConfig:
    """FHE parameter configuration.

    Attributes:
        scheme: The FHE scheme to use.
        poly_modulus_degree: Polynomial modulus degree (power of 2).
        coeff_mod_bit_sizes: Coefficient modulus bit sizes for CKKS.
        global_scale: Global scale for CKKS encoding.
        security_level: Security level in bits (128, 192, 256).
    """

    scheme: FHEScheme = FHEScheme.CKKS
    poly_modulus_degree: int = 8192
    coeff_mod_bit_sizes: list[int] | None = None
    global_scale: float | None = None
    security_level: int = 128

    def __post_init__(self) -> None:
        if self.coeff_mod_bit_sizes is None:
            # Default for CKKS with decent multiplicative depth
            self.coeff_mod_bit_sizes = [60, 40, 40, 60]
        if self.global_scale is None:
            self.global_scale = 2**40


@dataclass
class KeyBundle:
    """Bundle of FHE keys.

    Attributes:
        secret_key: Secret key bytes (CLIENT-ONLY, never send to server).
        public_key: Public key bytes.
        eval_keys: Evaluation keys (relinearization + galois keys).
        context: Serialized context/parameters.
    """

    secret_key: bytes
    public_key: bytes
    eval_keys: bytes
    context: bytes


class InferenceModel(Protocol):
    """Protocol for models that can run on encrypted data."""

    @property
    def model_id(self) -> str:
        """Unique identifier for this model."""
        ...

    @property
    def input_dim(self) -> int:
        """Expected input dimension."""
        ...

    @property
    def output_dim(self) -> int:
        """Output dimension."""
        ...

    def eval_encrypted(self, backend: "FHEBackend", ciphertext: Any) -> Any:
        """Run inference on encrypted input.

        Args:
            backend: The FHE backend to use for operations.
            ciphertext: Encrypted input (backend-specific type).

        Returns:
            Encrypted output (backend-specific type).
        """
        ...


class FHEBackend(ABC):
    """Abstract FHE backend interface.

    This is the core abstraction layer that allows swapping FHE implementations.
    To add a new backend (e.g., MOAI kernels), implement this interface.
    """

    @property
    @abstractmethod
    def scheme(self) -> FHEScheme:
        """The FHE scheme this backend implements."""
        ...

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable name of this backend."""
        ...

    @abstractmethod
    def setup_context(self, config: ParamsConfig) -> bytes:
        """Set up FHE context/parameters.

        Args:
            config: Parameter configuration.

        Returns:
            Serialized context bytes.
        """
        ...

    @abstractmethod
    def keygen(self, context: bytes) -> KeyBundle:
        """Generate FHE keys from context.

        Args:
            context: Serialized context from setup_context().

        Returns:
            KeyBundle with secret_key, public_key, eval_keys, and context.
        """
        ...

    @abstractmethod
    def encrypt(
        self,
        context: bytes,
        secret_key: bytes,
        plaintext: npt.NDArray[np.floating[Any]],
    ) -> bytes:
        """Encrypt a plaintext vector.

        Args:
            context: Serialized context.
            secret_key: Secret key bytes.
            plaintext: Input vector to encrypt.

        Returns:
            Serialized ciphertext bytes.
        """
        ...

    @abstractmethod
    def decrypt(
        self,
        context: bytes,
        secret_key: bytes,
        ciphertext: bytes,
        output_size: int,
    ) -> npt.NDArray[np.floating[Any]]:
        """Decrypt a ciphertext to plaintext vector.

        Args:
            context: Serialized context.
            secret_key: Secret key bytes.
            ciphertext: Serialized ciphertext.
            output_size: Expected output vector size.

        Returns:
            Decrypted plaintext vector.
        """
        ...

    @abstractmethod
    def load_ciphertext(self, context: bytes, eval_keys: bytes, ciphertext: bytes) -> Any:
        """Load a ciphertext for evaluation.

        Args:
            context: Serialized context.
            eval_keys: Evaluation keys.
            ciphertext: Serialized ciphertext.

        Returns:
            Backend-specific ciphertext object ready for operations.
        """
        ...

    @abstractmethod
    def serialize_ciphertext(self, ciphertext: Any) -> bytes:
        """Serialize a ciphertext object to bytes.

        Args:
            ciphertext: Backend-specific ciphertext object.

        Returns:
            Serialized ciphertext bytes.
        """
        ...

    @abstractmethod
    def eval(
        self,
        context: bytes,
        eval_keys: bytes,
        ciphertext: bytes,
        model: InferenceModel,
    ) -> bytes:
        """Run encrypted inference.

        Args:
            context: Serialized context.
            eval_keys: Evaluation keys for homomorphic operations.
            ciphertext: Serialized input ciphertext.
            model: Model to run inference with.

        Returns:
            Serialized output ciphertext.
        """
        ...

    # Convenience operations for models to use during eval
    @abstractmethod
    def add_plain(self, ciphertext: Any, plaintext: npt.NDArray[np.floating[Any]]) -> Any:
        """Add plaintext vector to ciphertext."""
        ...

    @abstractmethod
    def multiply_plain(self, ciphertext: Any, plaintext: npt.NDArray[np.floating[Any]]) -> Any:
        """Multiply ciphertext by plaintext vector."""
        ...

    @abstractmethod
    def dot_plain(
        self, ciphertext: Any, plaintext: npt.NDArray[np.floating[Any]]
    ) -> Any:
        """Compute dot product of ciphertext with plaintext vector."""
        ...

    @abstractmethod
    def matmul_plain(
        self,
        ciphertext: Any,
        weight_matrix: npt.NDArray[np.floating[Any]],
    ) -> Any:
        """Matrix multiply: ciphertext vector × plaintext weight matrix.

        Args:
            ciphertext: Encrypted input vector.
            weight_matrix: Plaintext weight matrix.

        Returns:
            Encrypted output vector.
        """
        ...

    @abstractmethod
    def square(self, ciphertext: Any) -> Any:
        """Square the ciphertext (polynomial activation)."""
        ...
