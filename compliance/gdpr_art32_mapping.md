# GDPR Article 32: Security of Processing

**Service:** MOAI FHE Inference  
**Scope:** Processing of Robot/Sensor Data (Potential PII)

## 1. Context & Risk Analysis
*Article 32(1): Taking into account the state of the art, the costs of implementation and the nature, scope, context and purposes of processing...*

**Assessment:** MOAI processes data using Fully Homomorphic Encryption (FHE). The server **never** possesses the private key and thus cannot decrypt any personal data. This represents the "state of the art" in privacy-preserving computation.

## 2. Specific Measures (Art. 32(1)(a)-(d))

### (a) Pseudonymisation and Encryption
| Requirement | Evidence / Implementation |
|---|---|
| Encryption of personal data | All inputs are CKKS encrypted *before* leaving the client. See [Client SDK](/moai_client_sdk/client.py). |
| Transport encryption | mTLS enforced for all gRPC connections. See [Server Config](/moai_service/grpc_server.py). |

### (b) Confidentiality, Integrity, Availability (CIA)
| Requirement | Evidence / Implementation |
|---|---|
| Confidentiality | Private keys remain on client device. Server memory dump yields only ciphertext. [Threat Model](/security/threat_model.md). |
| Integrity | FHE operations are mathematically deterministic (within CKKS error bounds). Integrity checks in [Integration Tests](/tests/test_integration.py). |
| Availability | Kubernetes HA deployment (ReplicaSet). [Helm Chart](/deploy/helm/moai-service/values.yaml). |
| Resilience | Auto-scaling and self-healing via K8s. Capacity tests in [Benchmarks](/bench/reports/report.md). |

### (c) Restoration of Availability
| Requirement | Evidence / Implementation |
|---|---|
| Restore access in timely manner | [Ops Runbook](/docs/ops_runbook.md) defines MTTR targets and restore procedures. |

### (d) Regular Testing of Effectiveness
| Requirement | Evidence / Implementation |
|---|---|
| Process for testing | Continuous Integration runs security & correctness tests on every commit. [CI Config](/.github/workflows/ci.yml). |
| Benchmark suite | Automated `make bench_all` runs S0-S7 scenarios regularly. |
