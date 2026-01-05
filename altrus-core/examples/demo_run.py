import time
from altrus.core.kernel import AltrusKernel

print("🚀 Starting Altrus kernel with observability...")

kernel = AltrusKernel()

print("📊 Metrics available at http://localhost:8000/metrics")

# Keep process alive
while True:
    time.sleep(1)
