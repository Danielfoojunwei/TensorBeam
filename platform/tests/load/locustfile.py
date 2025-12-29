import http.client
import json
import random
import time
from locust import HttpUser, task, between

class MOAIUser(HttpUser):
    wait_time = between(1, 3)

    def on_start(self):
        """Register tenant on startup."""
        self.tenant_id = f"load-test-{random.randint(1000,9999)}"
        res = self.client.post("/tenants", params={"name": self.tenant_id})
        if res.status_code == 200:
            self.tenant_id_db = res.json()["id"]
        else:
            self.tenant_id_db = "unknown"

    @task(3)
    def submit_job(self):
        """Submit inference job."""
        if self.tenant_id_db == "unknown":
            return
            
        self.client.post("/jobs", 
            params={
                "tenant_id": self.tenant_id_db, 
                "model_id": "bert-base-v1"
            },
            name="/jobs (submit)"
        )

    @task(1)
    def check_health(self):
        self.client.get("/healthz", name="/healthz")

    @task(1)
    def view_job_status(self):
        # In real test, would store job_ids from submit_job
        pass
