"""Linear and MLP classifiers for FHE inference.

These models run on encrypted inputs:
- Linear: logits = W @ x + b
- MLP: logits = W2 @ relu_approx(W1 @ x + b1) + b2

Use cases (slow-loop cognition):
- Compliance/risk gates
- SOP reranker
- Sensitive incident classification
- Approvals/constraints checking

The models use plaintext weights but encrypted inputs,
providing input confidentiality while leveraging known model weights.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from fhe_inference_service.core.backend import FHEBackend


@dataclass
class LinearClassifier:
    """Linear classifier for encrypted inference.

    Computes: logits = W @ x + b

    Where:
    - x: encrypted input vector (embedding)
    - W: plaintext weight matrix
    - b: plaintext bias vector
    - logits: encrypted output (decrypt on client, apply softmax)

    Attributes:
        model_id: Unique identifier.
        weights: Weight matrix of shape (input_dim, output_dim).
        bias: Bias vector of shape (output_dim,).
    """

    model_id: str
    weights: npt.NDArray[np.floating[Any]]  # (input_dim, output_dim)
    bias: npt.NDArray[np.floating[Any]]  # (output_dim,)

    @property
    def input_dim(self) -> int:
        return int(self.weights.shape[0])

    @property
    def output_dim(self) -> int:
        return int(self.weights.shape[1])

    def forward_plaintext(
        self, x: npt.NDArray[np.floating[Any]]
    ) -> npt.NDArray[np.floating[Any]]:
        """Forward pass on plaintext (for testing/comparison)."""
        return x @ self.weights + self.bias

    def eval_encrypted(self, backend: "FHEBackend", ciphertext: Any) -> Any:
        """Run encrypted inference.

        Args:
            backend: FHE backend for operations.
            ciphertext: Encrypted input vector.

        Returns:
            Encrypted logits.
        """
        # Matrix multiply: ct @ W
        result = backend.matmul_plain(ciphertext, self.weights)

        # Add bias: result + b
        result = backend.add_plain(result, self.bias)

        return result


@dataclass
class MLPClassifier:
    """2-layer MLP classifier with polynomial activation.

    Computes:
        h = W1 @ x + b1
        h_act = h^2  (polynomial approximation of activation)
        logits = W2 @ h_act + b2

    The square activation (x²) is FHE-friendly:
    - Single multiplication (polynomial degree 2)
    - No complex approximations needed
    - Reasonable approximation to ReLU for bounded inputs

    Attributes:
        model_id: Unique identifier.
        weights1: First layer weights (input_dim, hidden_dim).
        bias1: First layer bias (hidden_dim,).
        weights2: Second layer weights (hidden_dim, output_dim).
        bias2: Second layer bias (output_dim,).
    """

    model_id: str
    weights1: npt.NDArray[np.floating[Any]]  # (input_dim, hidden_dim)
    bias1: npt.NDArray[np.floating[Any]]  # (hidden_dim,)
    weights2: npt.NDArray[np.floating[Any]]  # (hidden_dim, output_dim)
    bias2: npt.NDArray[np.floating[Any]]  # (output_dim,)

    @property
    def input_dim(self) -> int:
        return int(self.weights1.shape[0])

    @property
    def hidden_dim(self) -> int:
        return int(self.weights1.shape[1])

    @property
    def output_dim(self) -> int:
        return int(self.weights2.shape[1])

    def forward_plaintext(
        self, x: npt.NDArray[np.floating[Any]]
    ) -> npt.NDArray[np.floating[Any]]:
        """Forward pass on plaintext (for testing/comparison)."""
        h = x @ self.weights1 + self.bias1
        h_act = h**2  # Polynomial activation
        return h_act @ self.weights2 + self.bias2

    def eval_encrypted(self, backend: "FHEBackend", ciphertext: Any) -> Any:
        """Run encrypted inference.

        Args:
            backend: FHE backend for operations.
            ciphertext: Encrypted input vector.

        Returns:
            Encrypted logits.
        """
        # First layer: W1 @ x + b1
        hidden = backend.matmul_plain(ciphertext, self.weights1)
        hidden = backend.add_plain(hidden, self.bias1)

        # Polynomial activation: h^2
        hidden = backend.square(hidden)

        # Second layer: W2 @ h_act + b2
        output = backend.matmul_plain(hidden, self.weights2)
        output = backend.add_plain(output, self.bias2)

        return output


def create_demo_linear_classifier(
    input_dim: int = 64,
    output_dim: int = 3,
    seed: int = 42,
) -> LinearClassifier:
    """Create a demo linear classifier with random weights.

    For production, replace with trained weights.

    Args:
        input_dim: Input embedding dimension.
        output_dim: Number of classes.
        seed: Random seed for reproducibility.

    Returns:
        LinearClassifier with random weights.
    """
    rng = np.random.default_rng(seed)

    # Initialize with small random weights (Xavier-like)
    scale = np.sqrt(2.0 / (input_dim + output_dim))
    weights = rng.normal(0, scale, (input_dim, output_dim)).astype(np.float64)
    bias = np.zeros(output_dim, dtype=np.float64)

    return LinearClassifier(
        model_id="demo-linear-classifier",
        weights=weights,
        bias=bias,
    )


def create_demo_mlp_classifier(
    input_dim: int = 64,
    hidden_dim: int = 32,
    output_dim: int = 3,
    seed: int = 42,
) -> MLPClassifier:
    """Create a demo MLP classifier with random weights.

    Uses smaller hidden dimension to fit within CKKS slots.

    Args:
        input_dim: Input embedding dimension.
        hidden_dim: Hidden layer dimension.
        output_dim: Number of classes.
        seed: Random seed for reproducibility.

    Returns:
        MLPClassifier with random weights.
    """
    rng = np.random.default_rng(seed)

    # First layer (Xavier)
    scale1 = np.sqrt(2.0 / (input_dim + hidden_dim))
    weights1 = rng.normal(0, scale1, (input_dim, hidden_dim)).astype(np.float64)
    bias1 = np.zeros(hidden_dim, dtype=np.float64)

    # Second layer (Xavier)
    scale2 = np.sqrt(2.0 / (hidden_dim + output_dim))
    weights2 = rng.normal(0, scale2, (hidden_dim, output_dim)).astype(np.float64)
    bias2 = np.zeros(output_dim, dtype=np.float64)

    return MLPClassifier(
        model_id="demo-mlp-classifier",
        weights1=weights1,
        bias1=bias1,
        weights2=weights2,
        bias2=bias2,
    )
