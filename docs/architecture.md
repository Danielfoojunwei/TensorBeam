# MOAI FHE Inference System - Architecture

## System Overview

MOAI is a production-ready FHE (Fully Homomorphic Encryption) inference system for privacy-critical robotics operations. It enables encrypted inference where the cloud never sees plaintext data.

## Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                              ROBOT/EDGE (Trusted)                            │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │                       robops-edge-agent                                │  │
│  │                                                                        │  │
│  │  ┌─────────────┐   ┌─────────────┐   ┌─────────────────────────────┐  │  │
│  │  │ Platform    │   │ Adapter     │   │ moai-client-sdk            │  │  │
│  │  │ Connector   │──▶│ Plugin      │──▶│                             │  │  │
│  │  │ (ROS2/HTTP) │   │             │   │ • create_context()          │  │  │
│  │  └─────────────┘   └─────────────┘   │ • keygen() → sk LOCAL ONLY  │  │  │
│  │        │                             │ • encrypt(embedding)        │  │  │
│  │        │                             │ • submit_job(ciphertext)    │  │  │
│  │        ▼                             │ • decrypt(result)           │  │  │
│  │  ┌─────────────┐                     └─────────────────────────────┘  │  │
│  │  │ Writeback   │◀─────────────────────────────────────────────────────┤  │
│  │  │ (decisions) │                                                      │  │
│  │  └─────────────┘                                                      │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ gRPC + mTLS
                                    │ (ciphertexts + eval keys only)
                                    ↓
┌──────────────────────────────────────────────────────────────────────────────┐
│                         CLOUD/ON-PREM (Untrusted)                            │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │                       moai-service                                     │  │
│  │                                                                        │  │
│  │  ┌─────────────┐   ┌─────────────┐   ┌─────────────────────────────┐  │  │
│  │  │ gRPC API    │   │ Job Queue   │   │ Worker Pool                 │  │  │
│  │  │             │──▶│ (Redis opt) │──▶│ • Batch scheduling          │  │  │
│  │  │ • Register  │   │             │   │ • Tenant isolation          │  │  │
│  │  │ • Submit    │   │             │   │                             │  │  │
│  │  │ • Fetch     │   └─────────────┘   └─────────────────────────────┘  │  │
│  │  └─────────────┘                               │                      │  │
│  │                                                ▼                      │  │
│  │                          ┌─────────────────────────────────────────┐  │  │
│  │                          │ moai-core (FHE Engine)                  │  │  │
│  │                          │                                         │  │  │
│  │                          │ • Backend: TenSEAL (MVP) / OpenFHE (v2) │  │  │
│  │                          │ • CKKS operations                       │  │  │
│  │                          │ • Linear/MLP classifier inference       │  │  │
│  │                          │ • Model registry                        │  │  │
│  │                          └─────────────────────────────────────────┘  │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│  Server NEVER sees: Secret Key, Plaintext Data                               │
│  Server DOES see:   Eval Keys, Ciphertexts, Model Weights                    │
└──────────────────────────────────────────────────────────────────────────────┘
```

## Component Details

### moai-core
FHE computation engine with swappable backends:
- **TenSEAL Backend** (MVP): CKKS scheme, Python-first
- **OpenFHE Backend** (v2): Full control, C++ performance

### moai-service
gRPC service layer:
- Job queue with optional Redis persistence
- Batching for throughput optimization
- Per-tenant eval key isolation
- OpenTelemetry instrumentation

### moai-client-sdk
Client-side encryption library:
- Context and key generation (secret key stays local)
- Column packing for embeddings
- Job submission and result retrieval
- Decryption (client-side only)

### robops-edge-agent
ROS2/HTTP integration layer:
- Platform adapters (pluggable)
- Event normalization
- Decision writeback

## Adapter Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Adapter Plugin Interface                     │
├─────────────────────────────────────────────────────────────────┤
│ class RobOpsAdapter(ABC):                                       │
│     def connect() -> None                                       │
│     def ingest_event(raw) -> RobotOpsEvent                      │
│     def to_inference_request(event) -> InferenceRequest         │
│     def writeback(decision: InferenceDecision) -> None          │
└─────────────────────────────────────────────────────────────────┘
         ▲           ▲           ▲           ▲           ▲
         │           │           │           │           │
    ┌────┴────┐ ┌────┴────┐ ┌────┴────┐ ┌────┴────┐ ┌────┴────┐
    │ ROS2   │ │Open-RMF │ │openTCS  │ │ Cloud   │ │ Webhook │
    │Generic │ │         │ │         │ │Robotics │ │ Generic │
    └─────────┘ └─────────┘ └─────────┘ └─────────┘ └─────────┘
```

## Data Flow

### Incident Triage Flow
```
1. Robot event (ROS2/API) ──────▶ Edge Agent
2. Edge normalizes to RobotOpsEvent
3. Client SDK: tokenize ──▶ embed ──▶ encrypt
4. Submit encrypted embedding ──────▶ MOAI Service
5. FHE inference (linear classifier)
6. Return encrypted logits ──────▶ Edge Agent
7. Client SDK: decrypt ──▶ softmax ──▶ argmax
8. Writeback decision (ROS2 topic / API callback)
```

### SOP Rerank Flow
```
1. Query + SOP candidates (from local RAG)
2. For each (query, candidate) pair:
   - Embed both locally
   - Encrypt embeddings
3. Submit batch to MOAI
4. FHE cross-encoder scoring
5. Decrypt scores, rank
6. Writeback ranked list
```

## Security Model

| Asset | Status | Notes |
|-------|--------|-------|
| Input embeddings | 🔒 Encrypted | CKKS ciphertext |
| Output logits | 🔒 Encrypted | Client decrypts |
| Secret key | 🔒 Local only | Never transmitted |
| Eval keys | ⚠️ Transmitted | No decrypt capability |
| Model weights | ⚠️ Plaintext | Server has access |
| Access patterns | ⚠️ Visible | Timing, frequency |

## Latency Model

| Operation | Expected Latency | Notes |
|-----------|------------------|-------|
| Key generation | 2-5 seconds | One-time per session |
| Encryption | 10-50 ms | Per embedding |
| Linear inference | 100-500 ms | W @ x + b |
| Decryption | 5-20 ms | Per result |
| **Total RTT** | **~0.5-2 seconds** | Without network |

This is **slow-loop** cognition - suitable for:
- Incident triage (seconds acceptable)
- SOP reranking (seconds acceptable)
- Compliance scoring (seconds acceptable)

**NOT suitable for**:
- Real-time robot control (requires <50ms)
- Vision-to-action VLA loops
