"""Fast-Lane Inference Demo.

Demonstrates hybrid architecture:
- Fast lane: Plaintext student model (latency < 1ms)
- Slow lane: FHE teacher model (latency ~1s)
"""

from __future__ import annotations

import sys
import os

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import time
import numpy as np
from moai_core.polynomial_ops import sigmoid_polynomial, gelu_polynomial
from moai_core.simd_batching import pack_embeddings_column

def fast_lane_student_model(embedding: np.ndarray) -> tuple[float, float]:
    """Plaintext student model (simulated).
    
    Returns: (score, confidence)
    """
    # Simulate a lightweight model
    score = float(sigmoid_polynomial(np.sum(embedding * 0.1)))
    confidence = abs(score - 0.5) * 2  # High near 0 or 1
    return score, confidence

def slow_lane_fhe_model(embedding: np.ndarray) -> float:
    """FHE teacher model (simulated latency)."""
    # Simulate FHE encryption + computation + decryption
    time.sleep(1.0)  # 1 second latency
    return float(sigmoid_polynomial(np.sum(embedding * 0.15)))  # Slightly better model

def hybrid_inference(embedding: np.ndarray, threshold: float = 0.8) -> dict:
    """Hybrid inference pipeline."""
    start = time.time()
    
    # 1. Try fast lane
    score, confidence = fast_lane_student_model(embedding)
    
    if confidence > threshold:
        return {
            "mode": "fast_lane",
            "score": score,
            "latency_ms": (time.time() - start) * 1000,
            "confidence": confidence
        }
        
    # 2. Fallback to slow lane (FHE)
    # In real MOAI, this would call client.infer()
    fhe_score = slow_lane_fhe_model(embedding)
    
    return {
        "mode": "slow_lane_fhe",
        "score": fhe_score,
        "latency_ms": (time.time() - start) * 1000, 
        "confidence": 1.0  # Assumed ground truth
    }

def main():
    print("=" * 60)
    print("MOAI Hybrid Inference Demo (Fast Lane Optimization)")
    print("=" * 60)
    
    np.random.seed(42)
    
    # Case 1: Clear signal (Fast Lane)
    emb_clear = np.ones(64) * 0.5
    res = hybrid_inference(emb_clear)
    print(f"\n[Case 1] Clear Input")
    print(f"  Mode: {res['mode']}")
    print(f"  Latency: {res['latency_ms']:.2f} ms")
    print(f"  Confidence: {res['confidence']:.2f}")
    
    # Case 2: Ambiguous signal (Slow Lane)
    emb_ambiguous = np.random.randn(64) * 0.1
    res = hybrid_inference(emb_ambiguous)
    print(f"\n[Case 2] Ambiguous Input")
    print(f"  Mode: {res['mode']}")
    print(f"  Latency: {res['latency_ms']:.2f} ms")
    print(f"  Confidence: {1.0:.2f} (Teacher)")

if __name__ == "__main__":
    main()
