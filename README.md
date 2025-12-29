# TensorBeam: Enterprise FHE Inference Platform

**Production-Grade Fully Homomorphic Encryption (FHE) for Critical Robotics & AI**

TensorBeam (formerly Project MOAI) is a complete platform for **Encrypted Cognitive Inference**. It allows robotics fleets and sensitive AI systems to offload "Slow-Loop" cognitive tasks (e.g., incident triage, SOP reranking, medical diagnosis) to the cloud *without ever exposing the input data or results*.

**Key Capabilities:**
*   🔒 **Zero-Trust Inference**: Server *never* sees plaintext. Keys stay on the client.
*   ⚡ **Hybrid "Fast Lane"**: <1ms response for confident queries; FHE for ambiguous ones.
*   🚀 **SOTA Performance**: GPU Acceleration (H100/A100), SIMD Batching, and Polynomial Approximations.
*   🏢 **Multi-Tenant Control Plane**: Complete management API for tenants, keys, and models.
*   ✅ **Compliance Ready**: Evidence packs for SOC 2, GDPR, and HIPAA.

---

## 🏗️ System Architecture

TensorBeam is composed of three layers:

```mermaid
graph TD
    Client[Edge Robot / SDK] -->|Ciphertext + TLS| LB[Load Balancer]
    LB -->|mTLS| API[Platform API (Control Plane)]
    
    subgraph "TensorBeam Cloud"
        API -->|Enqueue| Queue[(Redis Job Queue)]
        Worker[GPU Worker] -->|Dequeue| Queue
        
        Worker -->|FFI| Engine[Core FHE Engine]
        Engine -->|CUDA| GPU[NVIDIA H100]
        
        API -->|Metadata| DB[(Postgres)]
        API -->|Storage| S3[(Encrypted Blob Store)]
    end
```

---

## 📦 Core Components

### 1. FHE Core Engine (`/moai_core`)
The mathematical heart of the system.
*   **Backends**: Supports TenSEAL (CPU/CKKS) and interfaces for FIDESlib/Phantom (GPU).
*   **SIMD Batching**: packs 50+ embeddings per ciphertext (98% latency reduction).
*   **Polynomial Ops**: Drop-in replacements for GELU, Softmax, LayerNorm.

### 2. Management Platform (`/platform`)
A white-label control plane for managing the lifecycle of encrypted inference.
*   **API**: FastAPI-based REST + gRPC endpoints (`/tenants`, `/jobs`, `/keys`).
*   **Security**: Multi-tenant isolation, automated key rotation, strict RBAC.
*   **Robustness**: Graceful degradation middleware, Chaos-tested reliability.
*   **Mock UI**: Reference Dashboard for tenant management.

### 3. Adapters (`/adapters`)
Pre-built integrations for the robotics ecosystem:
*   **ROS2**: `adapter_webrtc_ros` for video overlays.
*   **Fleet Managers**: InOrbit, Formant, Freedom Robotics support.
*   **Industrial**: OpenTCS, FogROS2 integration.

---

## 📊 Performance & Benchmarks

We maintain a rigorous benchmark suite (`/bench`) comparing TensorBeam against industry baselines.

| Metric | Plaintext Baseline | TensorBeam (FHE Lane) | TensorBeam (Hybrid) |
|---|---|---|---|
| **Latency** | 10ms | ~2.36 min (GPU) | **< 1ms** (Fast Lane) |
| **Throughput** | 1000 QPS | 0.007 QPS | **1000+ QPS** |
| **Privacy** | ❌ None | ✅ Provable | ✅ High Assurance |

*   **Benchmark Harness**: `python bench/runner.py --scenarios s0 s1`
*   **Optimization Guide**: [See Docs](docs/optimization_guide.md)

---

## 🛡️ Security & Compliance

TensorBeam is designed for regulated environments. The `/compliance` directory contains automated evidence mapping:

*   ✅ **SOC 2 Type II**: Mappings for Trusted Services Criteria (Access, Encryption).
*   ✅ **GDPR Art. 32**: Proof of encryption-at-rest and pseudonymization.
*   ✅ **HIPAA**: Technical Safeguards mapping (Audit controls, Integrity).
*   **Pentest Report**: [See Findings](platform/security/pentest_report.md).

---

## 🚀 Quickstart

### 1. Run the Platform (Docker Compose)
Deploy the full stack (API, DB, Worker, Mock UI) locally.

```bash
cd platform/deploy
docker-compose up -d
```
Access the dashboard at `http://localhost:3000` and API at `http://localhost:8000`.

### 2. Client SDK Example
```python
from moai_client_sdk import MOAIClient

# Initialize Client (Keys generated locally)
client = MOAIClient("http://localhost:8000", api_key="tenant-secret")

# Encrypt & Infer
# (Data is encrypted BEFORE network transmission)
prediction = client.infer(
    model_id="bert-sota-v1",
    inputs=["Is this anomaly critical?"] 
)

print(f"Result: {prediction} (Confidence: High)")
```

### 3. Run Verification Tests
```bash
# Run Unit Tests
make test

# Run Benchmark Suite
python bench/harness/runner.py --scenarios s0 s1 s7

# Run Chaos/Robustness Tests
python platform/tests/robustness/test_chaos.py
```

---

## 📂 Repository Structure

```
TensorBeam/
├── platform/               # Control Plane (Backend/Frontend/Deploy)
│   ├── backend/            # FastAPI Service
│   ├── deploy/             # K8s & Docker Compose
│   └── security/           # Pentest Plans
├── moai_core/              # FHE Engine (Math & Backends)
├── moai_client_sdk/        # Python Client Library
├── adapters/               # 11 Robotics Platform Integrations
├── bench/                  # Benchmark & Evidence Suite
├── compliance/             # SOC2/GDPR/HIPAA Artifacts
└── docs/                   # Architecture & Guides
```

## License
MIT License. Copyright (c) 2025 TensorBeam Team.
