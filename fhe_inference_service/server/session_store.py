"""Session store for evaluation keys.

Manages per-session FHE evaluation keys. The server stores eval keys
(galois + relinearization) but NEVER the secret key.

Current implementation: In-memory dict (ephemeral, single-node).
Future: Redis/persistent storage for multi-node deployments.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any

logger = logging.getLogger(__name__)


@dataclass
class SessionData:
    """Data stored for each session.

    Attributes:
        session_id: Unique session identifier.
        fhe_scheme: FHE scheme used (ckks, bfv, tfhe).
        context: Serialized public context (no secret key).
        eval_keys: Serialized evaluation keys.
        params: FHE parameters.
        created_at: Timestamp when session was created.
        last_used: Timestamp of last inference.
    """

    session_id: str
    fhe_scheme: str
    context: bytes
    eval_keys: bytes
    params: dict[str, Any]
    created_at: float = field(default_factory=time.time)
    last_used: float = field(default_factory=time.time)


class SessionStore:
    """Thread-safe in-memory session store.

    Stores evaluation keys keyed by session_id.
    Implements TTL-based expiration for security (ephemeral keys).

    Note: For production, replace with Redis or persistent store.
    """

    def __init__(self, default_ttl_seconds: int = 3600) -> None:
        """Initialize session store.

        Args:
            default_ttl_seconds: Default TTL for sessions (1 hour).
        """
        self._sessions: dict[str, SessionData] = {}
        self._lock = threading.RLock()
        self._default_ttl = default_ttl_seconds

    def register(
        self,
        session_id: str,
        fhe_scheme: str,
        context: bytes,
        eval_keys: bytes,
        params: dict[str, Any] | None = None,
    ) -> None:
        """Register a new session with evaluation keys.

        Args:
            session_id: Unique session identifier.
            fhe_scheme: FHE scheme (ckks, bfv, tfhe).
            context: Serialized public context.
            eval_keys: Serialized evaluation keys.
            params: FHE parameters.
        """
        with self._lock:
            self._sessions[session_id] = SessionData(
                session_id=session_id,
                fhe_scheme=fhe_scheme,
                context=context,
                eval_keys=eval_keys,
                params=params or {},
            )
            logger.info(f"Registered session: {session_id} (scheme: {fhe_scheme})")

    def get(self, session_id: str) -> SessionData | None:
        """Get session data by ID.

        Args:
            session_id: Session identifier.

        Returns:
            SessionData if found, None otherwise.
        """
        with self._lock:
            session = self._sessions.get(session_id)
            if session:
                # Check TTL
                if time.time() - session.created_at > self._default_ttl:
                    logger.info(f"Session expired: {session_id}")
                    del self._sessions[session_id]
                    return None
                # Update last used
                session.last_used = time.time()
            return session

    def delete(self, session_id: str) -> bool:
        """Delete a session.

        Args:
            session_id: Session to delete.

        Returns:
            True if deleted, False if not found.
        """
        with self._lock:
            if session_id in self._sessions:
                del self._sessions[session_id]
                logger.info(f"Deleted session: {session_id}")
                return True
            return False

    def cleanup_expired(self) -> int:
        """Remove all expired sessions.

        Returns:
            Number of sessions removed.
        """
        with self._lock:
            now = time.time()
            expired = [
                sid
                for sid, s in self._sessions.items()
                if now - s.created_at > self._default_ttl
            ]
            for sid in expired:
                del self._sessions[sid]
            if expired:
                logger.info(f"Cleaned up {len(expired)} expired sessions")
            return len(expired)

    def __len__(self) -> int:
        with self._lock:
            return len(self._sessions)

    def __contains__(self, session_id: str) -> bool:
        return self.get(session_id) is not None
