# FHE Inference Benchmarking Notes

## Performance Characteristics

### Expected Latencies (CPU, Single Thread)

| Operation | Typical Time | Notes |
|-----------|-------------|-------|
| Key generation | 2-5 seconds | One-time setup |
| Encryption (64-dim) | 10-50 ms | Per vector |
| Linear layer inference | 100-500 ms | W @ x + b |
| MLP inference | 500-2000 ms | Includes squaring |
| Decryption | 5-20 ms | Per output |

### Ciphertext Sizes

| Vector Size | Ciphertext Size | Compression |
|-------------|-----------------|-------------|
| 64 dims | ~130 KB | ~2000x expansion |
| 128 dims | ~130 KB | ~1000x expansion |
| 256 dims | ~130 KB | ~500x expansion |

> CKKS ciphertext size depends on poly_modulus_degree, not vector length.

### Scaling Factors

| Factor | Impact on Latency |
|--------|-------------------|
| Multiplicative depth | ~2x per level |
| Vector size | Sublinear (SIMD) |
| Poly modulus degree | ~4x for 2x increase |

---

## Benchmark Methodology

### Environment

- **CPU**: Record model and core count
- **Memory**: 16GB+ recommended
- **Python**: 3.11+
- **TenSEAL**: Latest stable

### Metrics Collected

1. **Keygen time**: Context creation + key generation
2. **Encrypt time**: Mean over 10 encryptions
3. **Infer time**: Server-side FHE computation only
4. **Decrypt time**: Mean over 10 decryptions
5. **Ciphertext sizes**: Input and output

### Reproducibility

Run benchmarks with:
```bash
make bench
# or
uv run python bench/run_bench.py
```

Results saved to:
- `bench/results/benchmark_results.json` (raw data)
- `bench/results/benchmark_results.md` (formatted table)

---

## Known Limitations

1. **No GPU acceleration**: TenSEAL CKKS runs on CPU only
2. **Single-threaded**: Benchmarks use one thread
3. **Cold start**: First operation may be slower (JIT)
4. **Memory pressure**: Large poly_modulus_degree uses more RAM

---

## Comparison with MOAI

MOAI (H200 GPU) reported performance for BERT-base:
- ~2.36 minutes amortized (batched)
- Targets 128-token sequences

This baseline (CPU, linear classifier):
- ~0.5 seconds for linear layer
- ~2 seconds for MLP
- No batching optimization

**Gap**: ~100-1000x (expected - MOAI uses specialized GPU kernels)

---

## Future Optimizations

1. **Batching**: SIMD slot utilization
2. **Threading**: Parallel coefficient operations
3. **GPU backends**: When available for CKKS
4. **Precomputation**: Rotation key optimization
