import logging
import asyncio
from fastapi import Request
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

logger = logging.getLogger("moai.middleware")

class RobustnessMiddleware(BaseHTTPMiddleware):
    """Graceful degradation and error handling."""
    
    def __init__(self, app, degrade_mode: bool = False):
        super().__init__(app)
        self.degrade_mode = degrade_mode

    async def dispatch(self, request: Request, call_next):
        # 1. Degraded Mode Check
        if self.degrade_mode:
            if request.method == "POST" and request.url.path == "/jobs":
                # In degraded mode, reject new jobs but allow status checks
                return JSONResponse(
                    status_code=503, 
                    content={"error": "System in degraded mode", "retry_after": 60}
                )

        # 2. Global Exception Handler
        try:
            response = await call_next(request)
            return response
        except Exception as e:
            logger.exception("Global handler caught exception")
            return JSONResponse(
                status_code=500,
                content={"error": "Internal Server Error", "code": "INTERNAL_ERROR"}
            )
