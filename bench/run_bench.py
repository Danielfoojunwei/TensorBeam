"""FHE Inference Service Benchmarks.

Measures:
- Key generation time
- Encryption time
- Server-side inference time
- Decryption time
- Ciphertext sizes

Outputs:
- Console summary
- bench/results/benchmark_results.json
- bench/results/benchmark_results.md
"""

from __future__ import annotations

import json
import platform
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from fhe_inference_service.core.backend import ParamsConfig
from fhe_inference_service.core.backends.tenseal_ckks import TenSEALCKKSBackend
from fhe_inference_service.core.models.linear_model import (
    create_demo_linear_classifier,
    create_demo_mlp_classifier,
)


@dataclass
class BenchmarkResult:
    """Results from a benchmark run."""

    operation: str
    model_type: str
    input_dim: int
    output_dim: int
    mean_time_ms: float
    std_time_ms: float
    iterations: int
    ct_input_bytes: int | None = None
    ct_output_bytes: int | None = None


@dataclass
class BenchmarkSummary:
    """Summary of all benchmark results."""

    timestamp: str
    platform: str
    python_version: str
    backend: str
    poly_modulus_degree: int
    results: list[BenchmarkResult]


def run_benchmarks(
    num_iterations: int = 5,
    input_dim: int = 64,
    output_dim: int = 3,
) -> BenchmarkSummary:
    """Run full benchmark suite.

    Args:
        num_iterations: Number of iterations for timing.
        input_dim: Input embedding dimension.
        output_dim: Number of output classes.

    Returns:
        BenchmarkSummary with all results.
    """
    print("=" * 60)
    print("FHE Inference Service Benchmark")
    print("=" * 60)

    backend = TenSEALCKKSBackend()
    config = ParamsConfig()
    results: list[BenchmarkResult] = []

    # Prepare models
    linear_model = create_demo_linear_classifier(input_dim, output_dim)
    mlp_model = create_demo_mlp_classifier(input_dim, output_dim=output_dim)

    # Sample input
    sample_input = np.random.randn(input_dim).astype(np.float64)

    # =========================================================================
    # 1. Key Generation
    # =========================================================================
    print("\n[1/5] Key Generation...")
    keygen_times = []
    for _ in range(num_iterations):
        start = time.perf_counter()
        context = backend.setup_context(config)
        keys = backend.keygen(context)
        keygen_times.append((time.perf_counter() - start) * 1000)

    results.append(
        BenchmarkResult(
            operation="keygen",
            model_type="N/A",
            input_dim=input_dim,
            output_dim=output_dim,
            mean_time_ms=np.mean(keygen_times),
            std_time_ms=np.std(keygen_times),
            iterations=num_iterations,
        )
    )
    print(f"   Mean: {np.mean(keygen_times):.2f}ms ± {np.std(keygen_times):.2f}ms")

    # Use last context/keys for remaining tests
    context = backend.setup_context(config)
    keys = backend.keygen(context)

    # =========================================================================
    # 2. Encryption
    # =========================================================================
    print("\n[2/5] Encryption...")
    encrypt_times = []
    for _ in range(num_iterations):
        start = time.perf_counter()
        ct = backend.encrypt(context, keys.secret_key, sample_input)
        encrypt_times.append((time.perf_counter() - start) * 1000)

    ct_input_bytes = len(ct)
    results.append(
        BenchmarkResult(
            operation="encrypt",
            model_type="N/A",
            input_dim=input_dim,
            output_dim=output_dim,
            mean_time_ms=np.mean(encrypt_times),
            std_time_ms=np.std(encrypt_times),
            iterations=num_iterations,
            ct_input_bytes=ct_input_bytes,
        )
    )
    print(f"   Mean: {np.mean(encrypt_times):.2f}ms ± {np.std(encrypt_times):.2f}ms")
    print(f"   Ciphertext size: {ct_input_bytes:,} bytes")

    # =========================================================================
    # 3. Linear Model Inference
    # =========================================================================
    print("\n[3/5] Linear Model Inference (encrypted)...")
    linear_times = []
    ct_input = backend.encrypt(context, keys.secret_key, sample_input)

    for _ in range(num_iterations):
        start = time.perf_counter()
        ct_output = backend.eval(context, keys.eval_keys, ct_input, linear_model)
        linear_times.append((time.perf_counter() - start) * 1000)

    ct_output_bytes = len(ct_output)
    results.append(
        BenchmarkResult(
            operation="inference",
            model_type="linear",
            input_dim=input_dim,
            output_dim=output_dim,
            mean_time_ms=np.mean(linear_times),
            std_time_ms=np.std(linear_times),
            iterations=num_iterations,
            ct_input_bytes=ct_input_bytes,
            ct_output_bytes=ct_output_bytes,
        )
    )
    print(f"   Mean: {np.mean(linear_times):.2f}ms ± {np.std(linear_times):.2f}ms")
    print(f"   Output ciphertext: {ct_output_bytes:,} bytes")

    # =========================================================================
    # 4. MLP Model Inference
    # =========================================================================
    print("\n[4/5] MLP Model Inference (encrypted)...")

    # Use config with more depth for MLP
    mlp_config = ParamsConfig(
        poly_modulus_degree=8192,
        coeff_mod_bit_sizes=[60, 40, 40, 40, 60],
    )
    mlp_context = backend.setup_context(mlp_config)
    mlp_keys = backend.keygen(mlp_context)
    mlp_ct_input = backend.encrypt(mlp_context, mlp_keys.secret_key, sample_input * 0.1)

    mlp_times = []
    for _ in range(num_iterations):
        start = time.perf_counter()
        mlp_ct_output = backend.eval(mlp_context, mlp_keys.eval_keys, mlp_ct_input, mlp_model)
        mlp_times.append((time.perf_counter() - start) * 1000)

    mlp_ct_output_bytes = len(mlp_ct_output)
    results.append(
        BenchmarkResult(
            operation="inference",
            model_type="mlp",
            input_dim=input_dim,
            output_dim=output_dim,
            mean_time_ms=np.mean(mlp_times),
            std_time_ms=np.std(mlp_times),
            iterations=num_iterations,
            ct_input_bytes=len(mlp_ct_input),
            ct_output_bytes=mlp_ct_output_bytes,
        )
    )
    print(f"   Mean: {np.mean(mlp_times):.2f}ms ± {np.std(mlp_times):.2f}ms")
    print(f"   Output ciphertext: {mlp_ct_output_bytes:,} bytes")

    # =========================================================================
    # 5. Decryption
    # =========================================================================
    print("\n[5/5] Decryption...")
    decrypt_times = []
    for _ in range(num_iterations):
        start = time.perf_counter()
        decrypted = backend.decrypt(context, keys.secret_key, ct_output, output_dim)
        decrypt_times.append((time.perf_counter() - start) * 1000)

    results.append(
        BenchmarkResult(
            operation="decrypt",
            model_type="N/A",
            input_dim=input_dim,
            output_dim=output_dim,
            mean_time_ms=np.mean(decrypt_times),
            std_time_ms=np.std(decrypt_times),
            iterations=num_iterations,
        )
    )
    print(f"   Mean: {np.mean(decrypt_times):.2f}ms ± {np.std(decrypt_times):.2f}ms")

    # =========================================================================
    # Summary
    # =========================================================================
    summary = BenchmarkSummary(
        timestamp=time.strftime("%Y-%m-%d %H:%M:%S"),
        platform=platform.platform(),
        python_version=platform.python_version(),
        backend=backend.name,
        poly_modulus_degree=config.poly_modulus_degree,
        results=results,
    )

    return summary


def save_results(summary: BenchmarkSummary, output_dir: Path) -> None:
    """Save benchmark results to files."""
    output_dir.mkdir(parents=True, exist_ok=True)

    # Save JSON
    json_path = output_dir / "benchmark_results.json"
    with open(json_path, "w") as f:
        json.dump(asdict(summary), f, indent=2)
    print(f"\nSaved JSON results to: {json_path}")

    # Save Markdown
    md_path = output_dir / "benchmark_results.md"
    with open(md_path, "w") as f:
        f.write("# FHE Inference Benchmark Results\n\n")
        f.write(f"**Timestamp**: {summary.timestamp}\n")
        f.write(f"**Platform**: {summary.platform}\n")
        f.write(f"**Python**: {summary.python_version}\n")
        f.write(f"**Backend**: {summary.backend}\n")
        f.write(f"**Poly Modulus Degree**: {summary.poly_modulus_degree}\n\n")

        f.write("## Results\n\n")
        f.write("| Operation | Model | Input Dim | Mean (ms) | Std (ms) | CT In (bytes) | CT Out (bytes) |\n")
        f.write("|-----------|-------|-----------|-----------|----------|---------------|----------------|\n")

        for r in summary.results:
            ct_in = f"{r.ct_input_bytes:,}" if r.ct_input_bytes else "N/A"
            ct_out = f"{r.ct_output_bytes:,}" if r.ct_output_bytes else "N/A"
            f.write(
                f"| {r.operation} | {r.model_type} | {r.input_dim} | "
                f"{r.mean_time_ms:.2f} | {r.std_time_ms:.2f} | {ct_in} | {ct_out} |\n"
            )

    print(f"Saved Markdown results to: {md_path}")


def main() -> None:
    """Run benchmarks and save results."""
    summary = run_benchmarks(num_iterations=5)

    # Save results
    output_dir = Path(__file__).parent / "results"
    save_results(summary, output_dir)

    print("\n" + "=" * 60)
    print("Benchmark complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
