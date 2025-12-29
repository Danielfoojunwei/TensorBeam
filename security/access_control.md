# Access Control Policy

**Service:** MOAI FHE Inference

## 1. Principles
*   **Least Privilege**: Users/Services act with minimum necessary permissions.
*   **Authentication**: All access requires verification of identity (mTLS/JWT).
*   **Auditing**: All access events are logged.

## 2. User/Service Categories
1.  **Robots (Clients)**:
    *   Authenticated via mTLS Client Certificates.
    *   Permissions: `RegisterKey`, `Infer`, `GetModelInfo`.
    *   Isolation: Can only access their own sessions (enforced by `session_id`).
2.  **RobOps Platforms (Adapters)**:
    *   Authenticated via API Keys or mTLS.
    *   Permissions: `Infer` (delegated), `GetStatus`.
3.  **Administrators**:
    *   Authenticated via OIDC/SSO (Future).
    *   Permissions: System config, Metrics view.

## 3. Implementation
*   **Platform**: Kubernetes RBAC for infra access.
*   **Application**:
    *   Middleware checks mTLS headers.
    *   Application logic validates Session ownership.

## 4. Monitoring
*   Failed auth attempts trigger alerts (Potential Brute Force).
*   Unauthorized resource access attempts are logged with `WARN` level.
