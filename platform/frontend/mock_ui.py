"""Mock Frontend (Next.js inspired).

Since we are in a Python env, this generates basic HTML/Dashboard
simulating the White-Label UI.
"""

from fastapi.responses import HTMLResponse

DASHBOARD_HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>MOAI Control Plane</title>
    <style>
        body { font-family: sans-serif; margin: 0; padding: 20px; background: #f5f5f5; }
        .header { background: #333; color: white; padding: 15px; border-radius: 4px; }
        .card { background: white; padding: 20px; margin-top: 20px; border-radius: 4px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .status { padding: 5px 10px; border-radius: 12px; font-size: 0.8em; }
        .status.QUEUED { background: #eee; }
        .status.PROCESSING { background: #e3f2fd; color: #1976d2; }
        .status.COMPLETED { background: #e8f5e9; color: #2e7d32; }
    </style>
</head>
<body>
    <div class="header">
        <h1>MOAI Platform</h1>
        <p>Tenant: ACME Corp (White Label Mode)</p>
    </div>
    
    <div class="card">
        <h2>Submit Inference</h2>
        <form action="/jobs" method="post">
            <input type="text" name="tenant_id" value="tenant-123" readonly>
            <input type="text" name="model_id" value="model-bert-base">
            <button type="button" onclick="submitJob()">Submit Ciphertext</button>
        </form>
        <div id="result"></div>
    </div>
    
    <div class="card">
        <h2>Recent Jobs</h2>
        <table width="100%">
            <tr><th>ID</th><th>Model</th><th>Status</th><th>Time</th></tr>
            <tr><td>job-abc</td><td>bert-base</td><td><span class="status COMPLETED">COMPLETED</span></td><td>2s ago</td></tr>
        </table>
    </div>

    <script>
        async function submitJob() {
            // Mock submission
            const res = await fetch('/jobs?tenant_id=tenant-123&model_id=model-v1', {method: 'POST'});
            const data = await res.json();
            document.getElementById('result').innerText = 'Job Submitted: ' + data.job_id;
        }
    </script>
</body>
</html>
"""

def get_dashboard():
    return HTMLResponse(content=DASHBOARD_HTML)
