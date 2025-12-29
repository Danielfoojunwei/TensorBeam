# SOC 2 Trust Services Criteria (TSC) Mapping

**Service:** MOAI FHE Inference  
**Date:** {DATE}  
**Version:** 1.0

## CC6.1 - Access Control
*The entity restricts physical and logical access to assets to authorized users.*

| Point of Focus | MOAI Evidence Artifact | Status |
|---|---|---|
| Restrict access to authorized users | [Auth Config](/moai_service/grpc_server.py) (Lines 45-60: JWT/mTLS) | ✅ Implemented |
| Limit access to system admins | [Admin Policy](/security/access_control.md) | ⏳ Pending |

## CC6.7 - Transmission Security
*The entity restricts the transmission of confidential or sensitive information...*

| Point of Focus | MOAI Evidence Artifact | Status |
|---|---|---|
| Use of encryption in transit | [mTLS Config](/moai_service/grpc_server.py); [TLS Test Report](/bench/reports/artifacts/tls_scan.json) | ✅ Implemented |
| Protection of keys | [Key Mgmt Policy](/security/key_management.md); [HSM Integration](/docs/key_management.md) | ✅ Documented |

## CC7.1 - Configuration/Vulnerability Management
*The entity monitors the configuration... for vulnerabilities.*

| Point of Focus | MOAI Evidence Artifact | Status |
|---|---|---|
| Vulnerability scans | [Trivy Report](/bench/reports/artifacts/trivy.json); [CI Pipeline](/.github/workflows/ci.yml) | ✅ Automated |
| Patch management | [Dockerfile](/deploy/docker/Dockerfile.moai-service) (Base image hygiene) | ✅ Managed |

## A1.2 - Availability / Authorization
*The entity authorizes, designs, develops... to meet availability objectives.*

| Point of Focus | MOAI Evidence Artifact | Status |
|---|---|---|
| Capacity planning | [Benchmark Report](/bench/reports/report.md) (Throughput scenarios) | ✅ Benchmarked |
| Backup/Recovery | [Disaster Recovery Plan](/docs/ops_runbook.md) | ✅ Documented |

## P3.1 - Personal Data Quality
*The entity collects and maintains accurate, up-to-date, complete... personal data.*

| Point of Focus | MOAI Evidence Artifact | Status |
|---|---|---|
| Data accuracy | [Integration Tests](/tests/test_integration.py) (Correctness check) | ✅ Verified |
| Minimization | [Architecture](/docs/architecture.md) (FHE - server sees NO data) | ✅ Inherently Secure |
