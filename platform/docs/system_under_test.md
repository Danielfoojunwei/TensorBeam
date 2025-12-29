# System Under Test: MOAI GPU Engine

**Version:** 1.0 (Hypothetical C++ CUDA Backend)
**Reference:** MOAI Paper (IACR ePrint 2025/991)

## 1. Engine Architecture
The core MOAI engine is a C++/CUDA application that performs bootstrapping-optimized CKKS inference.

### Build Artifacts
*   `libmoai_gpu.so`: Shared object exposing the C ABI.
*   `moai_test`: CLI test binary.

### C ABI Surface
The platform interacts with the engine via a stable Foreign Function Interface (FFI).

```c
typedef void* moai_handle_t;
typedef void* moai_model_t;

// Init Engine (allocates GPU memory pools, streams)
moai_handle_t moai_init(const char* config_json);

// Load Model (from bytes or path)
moai_model_t moai_load_model(moai_handle_t h, const char* model_path);

// Register Eval Keys (Tenant-scoped)
// keys_buf contains the serialized EvalKey + RelinearizationKey + GaloisKeys
int moai_register_keys(moai_handle_t h, const char* tenant_id, const void* keys_buf, size_t len);

// Submit Batch (Async/Blocking Hybrid)
// In: Packed ciphertext bytes (proto or raw)
// Out: Result ciphertext bytes
int moai_infer(
    moai_handle_t h, 
    moai_model_t m, 
    const char* tenant_id,
    const void* in_buf, size_t in_len,
    void** out_buf, size_t* out_len
);

// Cleanup
void moai_free_buffer(void* buf);
void moai_unload_model(moai_model_t m);
void moai_shutdown(moai_handle_t h);
```

## 2. Platform Integration Strategy

### "NativeBackend"
The production backend loads `libmoai_gpu.so` using `ctypes`.
*   **Memory Management**: Python allocates input buffers; C++ allocates output buffers (Python must free them).
*   **Concurrency**: The C++ engine manages its own CUDA streams. Python calls are thread-safe (release GIL).

### "MockBackend" (Current State)
For development/CI without H100 GPUs, the platform uses `moai_core` (TenSEAL/Python) wrapped to match the NativeBackend interface.

## 3. Determinism & Performance
*   **Determinism**: Run-to-run bitwise determinism is NOT guaranteed due to GPU atomic reductions, but error bounds are strictly checked.
*   **Latency**: Async submission is required. The engine batches requests internally or the platform batches them.
*   **Batching**: Platform enforces `(tenant, model, calibration)` grouping.

## 4. Security Boundaries
*   **Plaintext**: Engine NEVER sees plaintext. Input is `Ciphertext`, Output is `Ciphertext`.
*   **Keys**: `evk` is stored on disk/S3 encrypted. Loaded into GPU memory only during active session.
