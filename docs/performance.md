# Performance Guide

## Latency Model

MOAI is a **slow-loop** system. Expect minutes-level latency for complex models.

### Expected Latencies (CPU)

| Operation | Linear Classifier | MLP | BERT-like (future) |
|-----------|------------------|-----|---------------------|
| Key generation | 2-5s | 2-5s | 2-5s |
| Encryption | 10-50ms | 10-50ms | 100-500ms |
| Inference | 100-500ms | 500ms-2s | 2-5 min |
| Decryption | 5-20ms | 5-20ms | 50-200ms |
| **Total RTT** | **~0.5-2s** | **~1-3s** | **~3-6 min** |

### Ciphertext Sizes

| Vector Dim | Ciphertext | Wire Size (base64) |
|------------|-----------|-------------------|
| 64 | ~130 KB | ~175 KB |
| 128 | ~130 KB | ~175 KB |
| 512 | ~260 KB | ~350 KB |

Note: CKKS ciphertext size depends on poly_modulus_degree, not vector length.

## Optimization Strategies

### 1. Batching

Combine multiple requests to amortize FHE overhead:

```python
# Without batching: N × full_latency
for item in items:
    result = client.infer(model_id, embedding)

# With client-side batching: reduced overhead
batch = np.stack([embed1, embed2, embed3])
results = client.infer_batch(model_id, batch)  # Future API
```

### 2. Client-Side Embedding

Reduce what goes through FHE:

```
Expensive: text → tokenize → embed → encrypt → FHE_infer → decrypt
Better:    text → tokenize → embed (local) → encrypt → FHE_infer → decrypt

Local embedding is ~10ms, FHE on raw tokens is much slower.
```

### 3. Model Selection

Use smallest model that meets accuracy requirements:

| Model | Params | FHE Latency | Use Case |
|-------|--------|-------------|----------|
| Linear | ~200 | 100-500ms | Binary/simple classification |
| MLP (1 layer) | ~2K | 500ms-2s | Multi-class |
| Small BERT | ~14M | 2-5 min | Complex NLU (future) |

### 4. Parameter Tuning

Trade security for performance (with caution):

```python
# Fast (lower security ~80-bit)
config = ParamsConfig(poly_modulus_degree=4096)

# Standard (128-bit security)
config = ParamsConfig(poly_modulus_degree=8192)

# High security (slower, 192-bit)
config = ParamsConfig(poly_modulus_degree=16384)
```

## Throughput

### Single Instance
- Linear classifier: ~2-10 queries/second
- MLP: ~0.5-2 queries/second

### Scaling Out
```yaml
# Helm values for throughput
replicaCount: 10
moai:
  workerPoolSize: 8
```

Expected: near-linear scaling up to compute limits.

## Cost Drivers

| Factor | Impact | Mitigation |
|--------|--------|------------|
| Poly modulus degree | ~4x per doubling | Use minimum for security |
| Multiplicative depth | ~2x per level | Simpler models |
| Ciphertext count | Linear | Batch inputs |
| Network transfer | ~175KB per query | Compression (future) |

## Benchmarking

Run benchmarks:
```bash
python bench/run_bench.py
```

Results saved to:
- `bench/results/benchmark_results.json`
- `bench/results/benchmark_results.md`

## SLA Tiers (Recommended)

| Tier | Max Latency | Use Case |
|------|-------------|----------|
| Real-time | N/A | Not supported - use plaintext |
| Interactive | 5 seconds | Simple classification |
| Batch | 5 minutes | Complex NLU |
| Background | 30 minutes | Full transformer (future) |

## Future: GPU Acceleration

Current: CPU only (TenSEAL)

Future with MOAI kernels + GPU (H200):
- 10-100x speedup expected
- Amortized: ~2.36 min for BERT-base (per MOAI paper)
- Batched: even better amortization
