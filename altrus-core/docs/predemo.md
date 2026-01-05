

# 🎬 ALTRUS — End-to-End Demonstration

**Fault-Tolerant, Intent-Driven, Blockchain-Backed Middleware**

---

## 0️⃣ Pre-Demo Setup

### Terminal 1 – Kernel / Metrics

```bash
# ensure clean state
rm -rf ~/.altrus

# start CLI (this implicitly starts kernel + metrics server)
altrus modules list
```

### Terminal 2 – Metrics

```bash
# verify Prometheus metrics endpoint
curl http://localhost:8000/metrics
```

Explain:

> “This endpoint exposes real-time middleware metrics for observability.”

---

## 1️⃣ Module Registry & Lifecycle (CORE FEATURE)

### 1.1 Register a module

```bash
altrus modules add \
  --id nav1 \
  --name NavigationModule \
  --capabilities navigation.move
```

Explain:

> “Modules are dynamically registered with declared capabilities.”

---

### 1.2 Activate module

```bash
altrus modules activate --id nav1
altrus modules
```

Expected:

```
nav1 | ACTIVE | HEALTHY | navigation.move
```

Explain:

> “Activation timestamps and heartbeats are initialized.”

---

### 1.3 Observe metrics

```bash
curl http://localhost:8000/metrics | grep altrus_modules
```

Expected:

```
altrus_modules_total 1
altrus_modules_active 1
altrus_modules_failed 0
```

Explain:

> “Metrics reflect registry state in real time.”

---

## 2️⃣ Heartbeat & Health Evaluation (FAULT FOUNDATION)

### 2.1 Send heartbeat

```bash
altrus modules heartbeat --id nav1
```

Explain:

> “Modules can be monitored via heartbeat signals.”

---

### 2.2 Simulate heartbeat loss (failure)

```bash
sleep 40
altrus modules
```

Expected:

```
nav1 | FAILED | CRITICAL
```

Explain:

> “HealthMonitor + HealthEvaluator automatically detect failure.”

---

### 2.3 Observe metrics change

```bash
curl http://localhost:8000/metrics | grep altrus_modules
```

Expected:

```
altrus_modules_active 0
altrus_modules_failed 1
```

---

## 3️⃣ Blockchain Ledger (AUDIT BACKBONE)

### 3.1 View ledger

```bash
altrus ledger
```

Expected events:

```
MODULE_HEALTH_CHANGE
MODULE_STATE_CHANGE
MODULE_HEARTBEAT
```

Explain:

> “All state transitions are immutably logged to a blockchain-style ledger.”

---

## 4️⃣ Intent Injection (INTENT-DRIVEN CONTROL)

### 4.1 Inject intent

```bash
altrus intents inject --name NAVIGATE
altrus intents
```

Expected:

```
NAVIGATE | VALIDATED | HIGH | navigation.move
```

Explain:

> “Intents are validated and stored before execution.”

---

### 4.2 View pending intents

```bash
altrus intents pending
```

Explain:

> “Pending intents wait until system conditions allow execution.”

---

## 5️⃣ Intent Routing & Execution (POLICY-BASED)

### 5.1 Reactivate module

```bash
altrus modules activate --id nav1
```

### 5.2 Reconcile intents

```bash
altrus intents reconcile
altrus intents active
```

Expected:

```
NAVIGATE | HIGH | navigation.move | EXECUTING
```

Explain:

> “Routing policy selects the best module based on capability and health.”

---

### 5.3 Observe metrics

```bash
curl http://localhost:8000/metrics | grep intent
```

Expected:

```
altrus_intent_executing 1
```

---

## 6️⃣ Fault → Intent Preemption (KEY RESEARCH FEATURE)

### 6.1 Simulate failure during execution

```bash
sleep 40
altrus intents active
```

Expected:

```
None
```

Explain:

> “Executing intent was preempted due to module failure.”

---

### 6.2 Verify intent state

```bash
altrus intents list
```

Expected:

```
NAVIGATE | PREEMPTED | HIGH | navigation.move
```

---

### 6.3 Ledger proof

```bash
altrus ledger
```

Expected:

```
INTENT_PREEMPTED | MODULE_FAILURE
```

Explain:

> “Intent preemption is causally linked and immutably recorded.”

---

## 7️⃣ Adapter Demonstration (EXTENSIBILITY)

### 7.1 ML Adapter

```bash
python examples/ml_adapter_demo.py
altrus intents
```

Explain:

> “ML adapters can inject intents without CLI involvement.”

---

### 7.2 Adapter Architecture Explanation (no execution needed)

Say:

> “HTTP, gRPC, ROS adapters share a common Adapter base, enabling protocol-agnostic intent ingestion.”

(You do NOT need them running live — architecture is sufficient.)

---

## 8️⃣ Observability Dashboard (VISUAL IMPACT)

### 8.1 Prometheus (already shown)

```bash
curl http://localhost:8000/metrics
```

### 8.2 Grafana (optional but recommended)

* Open: `http://localhost:3000`
* Show:

  * Active modules
  * Failed modules
  * Executing intents

Say:

> “Grafana provides system-level observability, while the ledger provides forensic auditability.”

---

## 9️⃣ Ledger Web View (NEXT PHASE READY)

Say:

> “Ledger is UI-agnostic and can be exposed via REST or Web UI, which is planned as future work.”

(Optional: show JSON dump if implemented.)

