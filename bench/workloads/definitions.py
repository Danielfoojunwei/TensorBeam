"""Workload generators for Benchmarking.

Includes BERT-base workload definition.
"""

from dataclasses import dataclass
import numpy as np
from typing import Tuple, List

@dataclass
class WorkloadConfig:
    name: str 
    model_name: str
    seq_len: int
    batch_size: int
    input_shape: Tuple[int, ...]
    
class WorkloadGenerator:
    def __init__(self, config: WorkloadConfig):
        self.config = config
        
    def generate_batch(self) -> List[np.ndarray]:
        """Generate a random batch of inputs matching config."""
        # For BERT embeddings: [batch, seq_len, hidden=768]
        # MOAI typically takes pre-computed embeddings or token ids
        # Assuming FHE Model takes Encrypted Embeddings
        
        # Simulating embeddings input
        return [
            np.random.randn(*self.config.input_shape).astype(np.float32)
            for _ in range(self.config.batch_size)
        ]

BERT_BASE_WORKLOAD = WorkloadConfig(
    name="bert_base_s0",
    model_name="bert-base-uncased",
    seq_len=128,
    batch_size=256, # Amortized batch
    input_shape=(128, 768)
)

TINYBERT_WORKLOAD = WorkloadConfig(
    name="tinybert_s4",
    model_name="huawei-noah/TinyBERT_General_4L_312D",
    seq_len=128,
    batch_size=256,
    input_shape=(128, 312)
)
