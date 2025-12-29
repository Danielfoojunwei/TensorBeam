#!/usr/bin/env python3
"""FHE Inference Demo - MOAI-style Confidential Cognition.

This demo shows the complete end-to-end FHE inference pipeline:
1. Client generates keys locally (secret key never leaves)
2. Client registers evaluation keys with server
3. Client encrypts input embedding
4. Server runs encrypted inference
5. Client decrypts results

Use case: Slow-loop cognition for privacy-critical tasks
- Compliance/risk gates
- Incident classification
- SOP reranking
- Private scoring

The server NEVER sees plaintext data - only ciphertexts.
"""

from __future__ import annotations

import time

import numpy as np

from fhe_inference_service.client.sdk import FHEClient


def main() -> None:
    """Run the FHE inference demo."""
    print("=" * 70)
    print("FHE INFERENCE SERVICE DEMO")
    print("MOAI-style Confidential Cognition")
    print("=" * 70)

    server_url = "http://localhost:8000"
    print(f"\n[1/6] Connecting to server: {server_url}")

    with FHEClient(server_url=server_url) as client:
        # Health check
        try:
            health = client.health_check()
            print(f"       Server status: {health['status']}")
            print(f"       Backend: {health['backend']}")
            print(f"       Version: {health['version']}")
        except Exception as e:
            print(f"\n❌ ERROR: Cannot connect to server at {server_url}")
            print(f"   {e}")
            print("\n   Make sure the server is running:")
            print("   $ make run-server")
            print("   or")
            print("   $ uv run uvicorn fhe_inference_service.server.main:app --host 0.0.0.0 --port 8000")
            return

        # List available models
        print("\n[2/6] Available models:")
        models = client.list_models()
        for m in models:
            print(f"       - {m['model_id']}: {m['description']}")
            print(f"         Input: {m['input_dim']}, Output: {m['output_dim']}, Scheme: {m['scheme']}")

        # Setup keys
        print("\n[3/6] Generating FHE keys locally...")
        start = time.perf_counter()
        client.setup()
        keygen_time = (time.perf_counter() - start) * 1000
        print(f"       Key generation: {keygen_time:.2f}ms")
        print("       ✓ Secret key stays on client (NEVER sent to server)")

        # Register eval keys
        print("\n[4/6] Registering evaluation keys with server...")
        start = time.perf_counter()
        client.register_keys()
        register_time = (time.perf_counter() - start) * 1000
        print(f"       Registration: {register_time:.2f}ms")
        print(f"       Session ID: {client.session_id}")

        # Create sample embedding (simulating tokenization + embedding on client)
        print("\n[5/6] Creating sample embedding (client-side)...")
        np.random.seed(42)
        embedding = np.random.randn(64).astype(np.float64)
        embedding = embedding / np.linalg.norm(embedding)  # Normalize
        print(f"       Embedding shape: {embedding.shape}")
        print(f"       Embedding norm: {np.linalg.norm(embedding):.4f}")

        # Run encrypted inference
        print("\n[6/6] Running encrypted inference...")
        model_id = "demo-linear-classifier"
        print(f"       Model: {model_id}")
        print("       (Server processes encrypted data - sees NO plaintext)")

        start = time.perf_counter()
        result = client.infer(model_id, embedding)
        total_time = (time.perf_counter() - start) * 1000

        # Results
        print("\n" + "=" * 70)
        print("RESULTS")
        print("=" * 70)

        print(f"\n📊 Decrypted Logits: {result.logits}")
        print(f"🎯 Predicted Class: {result.predicted_class}")

        if result.probabilities is not None:
            print(f"📈 Probabilities: {result.probabilities}")
            confidence = result.probabilities[result.predicted_class]
            print(f"💪 Confidence: {confidence:.2%}")

        print("\n⏱️  Timing:")
        if result.metadata:
            print(f"   Server inference: {result.metadata['latency_ms']:.2f}ms")
            print(f"   Input ciphertext: {result.metadata['ct_input_bytes']:,} bytes")
            print(f"   Output ciphertext: {result.metadata['ct_output_bytes']:,} bytes")
        print(f"   Total round-trip: {total_time:.2f}ms")

        print("\n🔐 Security:")
        print("   ✓ Secret key: Never left client")
        print("   ✓ Plaintext input: Never sent to server")
        print("   ✓ Plaintext output: Decrypted only on client")
        print("   ✓ Server saw: Only ciphertext bytes")

        print("\n" + "=" * 70)
        print("Demo complete! The server learned NOTHING about your input.")
        print("=" * 70)

        # Comparison with plaintext (for verification)
        print("\n[Verification] Comparing with plaintext inference...")
        from fhe_inference_service.core.models.linear_model import create_demo_linear_classifier

        model = create_demo_linear_classifier()
        plaintext_logits = model.forward_plaintext(embedding)
        error = np.abs(result.logits - plaintext_logits)

        print(f"   Plaintext logits: {plaintext_logits}")
        print(f"   Encrypted logits: {result.logits}")
        print(f"   Absolute error:   {error}")
        print(f"   Max error:        {np.max(error):.6f}")

        if np.allclose(result.logits, plaintext_logits, rtol=1e-2, atol=1e-3):
            print("   ✓ Results match within tolerance!")
        else:
            print("   ⚠ Results differ (check CKKS parameters)")


if __name__ == "__main__":
    main()
