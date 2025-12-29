# HIPAA Security Rule Mapping (45 CFR Part 164)

**Service:** MOAI FHE Inference  
**Scope:** Processing of ePHI (if applicable)

## Administrative Safeguards (164.308)

| Section | Safeguard | Implementation / Evidence |
|---|---|---|
| **164.308(a)(1)** | **Security Management Process** | [Threat Model](/security/threat_model.md) used for risk analysis. |
| **164.308(a)(4)** | **Information Access Management** | Access restricted to FHE processing service accounts. No human access to raw data (it's encrypted anyway). |
| **164.308(a)(5)** | **Security Awareness Training** | *Organizational Policy (Out of Scope for Codebase)* |

## Physical Safeguards (164.310)

| Section | Safeguard | Implementation / Evidence |
|---|---|---|
| **164.310(d)(1)** | **Device and Media Controls** | Containers are ephemeral. No persistent local storage of ePHI. |

## Technical Safeguards (164.312)

| Section | Safeguard | Implementation / Evidence |
|---|---|---|
| **164.312(a)(1)** | **Access Control** | Unique User ID: mTLS certs identify clients unique ID. <br> Emergency Access: [Runbook](/docs/ops_runbook.md). |
| **164.312(b)** | **Audit Controls** | All gRPC requests logged with Metadata. [Observability](/moai_service/observability.py). Logs do NOT contain ePHI (encrypted). |
| **164.312(c)(1)** | **Integrity** | FHE ciphertexts protected against tampering (decryption fails if modified). ePHI is mathematically protected. |
| **164.312(d)** | **Person or Entity Authentication** | mTLS Mutual Authentication. Server verifies Client Cert. Client verifies Server Cert. |
| **164.312(e)(1)** | **Transmission Security** | Encryption (AES/TLS 1.3) + FHE (Content Encryption). Dual-layer protection. |

## Organizational Requirements (164.314)

| Section | Safeguard | Implementation / Evidence |
|---|---|---|
| **164.314(a)(2)(i)** | **BAA Contracts** | *Legal Requirement (Out of Scope for Codebase)* |
