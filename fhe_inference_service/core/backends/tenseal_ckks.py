"""TenSEAL CKKS Backend Implementation.

This module implements the FHE backend using TenSEAL with the CKKS scheme,
which supports approximate arithmetic on encrypted real-number vectors.

CKKS is ideal for ML inference because:
- Supports floating-point operations (add, multiply)
- Efficient for vector operations (SIMD packing)
- Suitable for neural network layers

Future: Swap this backend for MOAI transformer kernels when available.
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np
import numpy.typing as npt
import tenseal as ts

from fhe_inference_service.core.backend import (
    FHEBackend,
    FHEScheme,
    InferenceModel,
    KeyBundle,
    ParamsConfig,
)

logger = logging.getLogger(__name__)


class TenSEALCKKSBackend(FHEBackend):
    """TenSEAL CKKS backend for approximate arithmetic on encrypted vectors.

    This backend supports:
    - Encrypted vector addition/subtraction
    - Encrypted-plaintext multiplication
    - Matrix-vector multiplication for linear layers
    - Polynomial activations (square)

    Typical use case: Encrypted linear classifier or MLP for
    compliance gates, risk scoring, incident classification.
    """

    @property
    def scheme(self) -> FHEScheme:
        return FHEScheme.CKKS

    @property
    def name(self) -> str:
        return "TenSEAL-CKKS"

    def setup_context(self, config: ParamsConfig) -> bytes:
        """Set up TenSEAL CKKS context.

        Args:
            config: Parameter configuration.

        Returns:
            Serialized context bytes (without keys).
        """
        if config.scheme != FHEScheme.CKKS:
            raise ValueError(f"TenSEAL CKKS backend requires CKKS scheme, got {config.scheme}")

        context = ts.context(
            ts.SCHEME_TYPE.CKKS,
            poly_modulus_degree=config.poly_modulus_degree,
            coeff_mod_bit_sizes=config.coeff_mod_bit_sizes,
        )
        context.global_scale = config.global_scale or 2**40

        # Generate keys (we'll extract them separately)
        context.generate_galois_keys()
        context.generate_relin_keys()

        # Serialize context with all keys
        return context.serialize(save_secret_key=True)

    def keygen(self, context: bytes) -> KeyBundle:
        """Generate keys from context.

        Note: TenSEAL generates keys during context creation.
        The context serialized with save_secret_key=True contains everything.
        We create a public version without the secret key for server use.
        """
        # Load context with secret key
        ctx = ts.context_from(context)

        # The full context (with secret key) is the "secret key" for our purposes
        secret_key_context = context  # This already has the secret key embedded

        # Create public context (no secret key) for server
        ctx_public = ctx.copy()
        ctx_public.make_context_public()
        public_context = ctx_public.serialize()

        # For TenSEAL, eval keys are part of the public context
        # The public context includes galois and relin keys but no secret key
        eval_keys = public_context

        return KeyBundle(
            secret_key=secret_key_context,  # Full context with secret key
            public_key=public_context,  # Public context (no secret key)
            eval_keys=eval_keys,  # Same as public context for TenSEAL
            context=context,  # Original full context
        )

    def encrypt(
        self,
        context: bytes,
        secret_key: bytes,  # noqa: ARG002 - TenSEAL uses context with embedded key
        plaintext: npt.NDArray[np.floating[Any]],
    ) -> bytes:
        """Encrypt a plaintext vector using CKKS.

        Args:
            context: Full context with secret key.
            secret_key: Secret key (embedded in context for TenSEAL).
            plaintext: Input vector to encrypt.

        Returns:
            Serialized encrypted vector.
        """
        ctx = ts.context_from(context)
        encrypted = ts.ckks_vector(ctx, plaintext.tolist())
        return encrypted.serialize()

    def decrypt(
        self,
        context: bytes,
        secret_key: bytes,  # noqa: ARG002 - embedded in context
        ciphertext: bytes,
        output_size: int,
    ) -> npt.NDArray[np.floating[Any]]:
        """Decrypt ciphertext to plaintext vector.

        Args:
            context: Full context with secret key.
            secret_key: Secret key (embedded in context).
            ciphertext: Serialized ciphertext.
            output_size: Expected output size.

        Returns:
            Decrypted vector.
        """
        ctx = ts.context_from(context)
        encrypted = ts.ckks_vector_from(ctx, ciphertext)
        decrypted = encrypted.decrypt()
        return np.array(decrypted[:output_size], dtype=np.float64)

    def load_ciphertext(
        self,
        context: bytes,
        eval_keys: bytes,  # noqa: ARG002 - bundled in context for TenSEAL
        ciphertext: bytes,
    ) -> ts.CKKSVector:
        """Load ciphertext for server-side evaluation.

        Args:
            context: Public context with eval keys.
            eval_keys: Evaluation keys (bundled in context).
            ciphertext: Serialized ciphertext.

        Returns:
            TenSEAL CKKSVector ready for operations.
        """
        ctx = ts.context_from(context)
        return ts.ckks_vector_from(ctx, ciphertext)

    def serialize_ciphertext(self, ciphertext: Any) -> bytes:
        """Serialize ciphertext to bytes."""
        if not isinstance(ciphertext, ts.CKKSVector):
            raise TypeError(f"Expected CKKSVector, got {type(ciphertext)}")
        return ciphertext.serialize()

    def eval(
        self,
        context: bytes,
        eval_keys: bytes,
        ciphertext: bytes,
        model: InferenceModel,
    ) -> bytes:
        """Run encrypted inference using the model.

        Args:
            context: Public context (no secret key).
            eval_keys: Evaluation keys.
            ciphertext: Encrypted input.
            model: Model to evaluate.

        Returns:
            Encrypted output.
        """
        # Load ciphertext with public context
        encrypted_input = self.load_ciphertext(context, eval_keys, ciphertext)

        # Run model's encrypted evaluation
        encrypted_output = model.eval_encrypted(self, encrypted_input)

        return self.serialize_ciphertext(encrypted_output)

    def add_plain(
        self,
        ciphertext: Any,
        plaintext: npt.NDArray[np.floating[Any]],
    ) -> ts.CKKSVector:
        """Add plaintext to ciphertext."""
        if not isinstance(ciphertext, ts.CKKSVector):
            raise TypeError(f"Expected CKKSVector, got {type(ciphertext)}")
        return ciphertext + plaintext.tolist()

    def multiply_plain(
        self,
        ciphertext: Any,
        plaintext: npt.NDArray[np.floating[Any]],
    ) -> ts.CKKSVector:
        """Element-wise multiply ciphertext by plaintext."""
        if not isinstance(ciphertext, ts.CKKSVector):
            raise TypeError(f"Expected CKKSVector, got {type(ciphertext)}")
        return ciphertext * plaintext.tolist()

    def dot_plain(
        self,
        ciphertext: Any,
        plaintext: npt.NDArray[np.floating[Any]],
    ) -> ts.CKKSVector:
        """Compute dot product with plaintext vector."""
        if not isinstance(ciphertext, ts.CKKSVector):
            raise TypeError(f"Expected CKKSVector, got {type(ciphertext)}")
        return ciphertext.dot(plaintext.tolist())

    def matmul_plain(
        self,
        ciphertext: Any,
        weight_matrix: npt.NDArray[np.floating[Any]],
    ) -> ts.CKKSVector:
        """Matrix multiply: ciphertext @ weight_matrix.

        For a linear layer: output = input @ W + b
        where input is encrypted and W is plaintext.

        Args:
            ciphertext: Encrypted input vector of shape (input_dim,).
            weight_matrix: Weight matrix of shape (input_dim, output_dim).

        Returns:
            Encrypted output vector of shape (output_dim,).
        """
        if not isinstance(ciphertext, ts.CKKSVector):
            raise TypeError(f"Expected CKKSVector, got {type(ciphertext)}")

        # TenSEAL's matmul: vector @ matrix
        return ciphertext.matmul(weight_matrix.tolist())

    def square(self, ciphertext: Any) -> ts.CKKSVector:
        """Square the ciphertext (polynomial activation x²).

        Note: This consumes one multiplicative depth level.
        """
        if not isinstance(ciphertext, ts.CKKSVector):
            raise TypeError(f"Expected CKKSVector, got {type(ciphertext)}")
        return ciphertext * ciphertext
