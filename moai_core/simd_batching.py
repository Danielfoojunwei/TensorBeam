"""SIMD Batching Utilities for FHE.

Implements efficient packing strategies to amortize FHE overhead
by processing multiple inputs in a single ciphertext.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import numpy.typing as npt


@dataclass
class BatchedInput:
    """Batched input for FHE inference."""

    packed_data: npt.NDArray[np.floating[Any]]
    batch_size: int
    embed_dim: int
    packing_mode: str  # "column" or "row"


def pack_embeddings_column(
    embeddings: list[npt.NDArray[np.floating[Any]]],
) -> BatchedInput:
    """Pack embeddings using column-major ordering.

    This packing allows SIMD operations across batch dimension.

    Args:
        embeddings: List of embedding vectors.

    Returns:
        BatchedInput with packed data.
    """
    batch_size = len(embeddings)
    embed_dim = embeddings[0].shape[0]

    # Column packing: interleave elements
    # [e1[0], e2[0], ..., en[0], e1[1], e2[1], ...]
    packed = np.zeros(batch_size * embed_dim, dtype=np.float64)

    for i, emb in enumerate(embeddings):
        for j in range(embed_dim):
            packed[j * batch_size + i] = emb[j]

    return BatchedInput(
        packed_data=packed,
        batch_size=batch_size,
        embed_dim=embed_dim,
        packing_mode="column",
    )


def pack_embeddings_row(
    embeddings: list[npt.NDArray[np.floating[Any]]],
) -> BatchedInput:
    """Pack embeddings using row-major ordering.

    This packing is simpler and works well for element-wise operations.

    Args:
        embeddings: List of embedding vectors.

    Returns:
        BatchedInput with packed data.
    """
    batch_size = len(embeddings)
    embed_dim = embeddings[0].shape[0]

    # Row packing: concatenate vectors
    # [e1[0], e1[1], ..., e1[d], e2[0], e2[1], ...]
    packed = np.concatenate(embeddings)

    return BatchedInput(
        packed_data=packed,
        batch_size=batch_size,
        embed_dim=embed_dim,
        packing_mode="row",
    )


def unpack_outputs_column(
    packed_output: npt.NDArray[np.floating[Any]],
    batch_size: int,
    output_dim: int,
) -> list[npt.NDArray[np.floating[Any]]]:
    """Unpack column-packed outputs.

    Args:
        packed_output: Packed output vector.
        batch_size: Number of inputs in batch.
        output_dim: Dimension of each output.

    Returns:
        List of individual output vectors.
    """
    outputs = []
    for i in range(batch_size):
        output = np.zeros(output_dim, dtype=np.float64)
        for j in range(output_dim):
            output[j] = packed_output[j * batch_size + i]
        outputs.append(output)
    return outputs


def unpack_outputs_row(
    packed_output: npt.NDArray[np.floating[Any]],
    batch_size: int,
    output_dim: int,
) -> list[npt.NDArray[np.floating[Any]]]:
    """Unpack row-packed outputs.

    Args:
        packed_output: Packed output vector.
        batch_size: Number of inputs in batch.
        output_dim: Dimension of each output.

    Returns:
        List of individual output vectors.
    """
    return [
        packed_output[i * output_dim : (i + 1) * output_dim]
        for i in range(batch_size)
    ]


def optimal_batch_size(
    poly_modulus_degree: int,
    embed_dim: int,
    safety_margin: float = 0.8,
) -> int:
    """Calculate optimal batch size for given parameters.

    Args:
        poly_modulus_degree: CKKS polynomial degree.
        embed_dim: Embedding dimension.
        safety_margin: Slot utilization factor (0-1).

    Returns:
        Recommended batch size.
    """
    # CKKS slots = poly_modulus_degree / 2
    max_slots = poly_modulus_degree // 2

    # Reserve some slots for computation overhead
    usable_slots = int(max_slots * safety_margin)

    # Each embedding needs embed_dim slots
    max_batch = usable_slots // embed_dim

    # Clamp to reasonable range
    return max(1, min(max_batch, 128))


class BatchProcessor:
    """Process inference requests in efficient batches."""

    def __init__(
        self,
        poly_modulus_degree: int = 8192,
        embed_dim: int = 64,
        batch_timeout_ms: float = 100.0,
    ) -> None:
        """Initialize batch processor.

        Args:
            poly_modulus_degree: CKKS parameter.
            embed_dim: Embedding dimension.
            batch_timeout_ms: Max wait time for batch fill.
        """
        self.poly_modulus_degree = poly_modulus_degree
        self.embed_dim = embed_dim
        self.batch_timeout_ms = batch_timeout_ms
        self.optimal_batch = optimal_batch_size(
            poly_modulus_degree, embed_dim
        )

        # Pending requests
        self._pending: list[npt.NDArray[np.floating[Any]]] = []

    def add_request(self, embedding: npt.NDArray[np.floating[Any]]) -> int:
        """Add request to pending batch.

        Args:
            embedding: Input embedding.

        Returns:
            Index in current batch.
        """
        idx = len(self._pending)
        self._pending.append(embedding)
        return idx

    def should_flush(self) -> bool:
        """Check if batch should be processed."""
        return len(self._pending) >= self.optimal_batch

    def flush(self) -> BatchedInput | None:
        """Flush pending requests as batch.

        Returns:
            Batched input or None if empty.
        """
        if not self._pending:
            return None

        batch = pack_embeddings_column(self._pending)
        self._pending = []
        return batch

    def get_pending_count(self) -> int:
        """Get number of pending requests."""
        return len(self._pending)
