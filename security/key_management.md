# Key Management Policy

**Service:** MOAI FHE Inference

## 1. Scope
This policy governs the generation, storage, usage, and destruction of cryptographic keys within the MOAI system.

## 2. Key Types

### 2.1 Fully Homomorphic Encryption (FHE) Keys
*   **Secret Key ($sk$)**: 
    *   **Generation**: Client-side only (random number generator).
    *   **Storage**: Volatile memory or secure enclave (TPM/TEE) on client. **NEVER** transmitted to server.
    *   **Rotation**: Per-session or per-mission basis.
*   **Evaluation Key ($evk$)**:
    *   **Generation**: Client-side derived from $sk$.
    *   **Storage**: Transmitted to server. Cached in `SessionStore` (Volatile/Redis).
    *   **Lifecycle**: Bound to session TTL. Automatically evicted.

### 2.2 Transport Layer Security (TLS) Keys
*   **Server Private Key**:
    *   **Storage**: Kubernetes Secret (encrypted at rest) or Vault.
    *   **Rotation**: 90 days (via Cert-Manager).
*   **Client Private Key**:
    *   **Storage**: Client secure storage.
    *   **Rotation**: 90 days.

## 3. Procedures

### 3.1 Key Generation
*   FHE keys must use cryptographically secure random number generators (`secrets` module or OS CSPRNG).
*   Keys must meet minimum security parameters (128-bit security level based on Homomorphic Encryption Standard).

### 3.2 Key Destruction
*   **Server**: $evk$ is deleted upon session expiry or explicit `DeleteSession` call. Memory overwrite/zeroization where possible.
*   **Client**: $sk$ should be wiped from memory after inference completion or agent termination.

## 4. Auditing
*   Key generation events (client-side logs).
*   Key registration events (server-side logs).
*   No logging of actual key material.
