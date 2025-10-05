# ALTRUS Core Middleware

**Adaptive Life-support & Therapeutic Robotic Unit System (ALTRUS)**
Fault‑Tolerant, Intent‑Driven, Blockchain‑Backed Middleware for Modular Assistive Robots

---

## 1. Project Overview

ALTRUS Core is the **middleware backbone** of the ALTRUS (Adaptive Life‑support & Therapeutic Robotic Unit System) project.
It acts as the *authoritative decision‑making, coordination, fault‑tolerance, and audit layer* between:

* Human / ML intent sources
* Robot functional modules (navigation, telemedicine, etc.)
* Fault detection & recovery mechanisms
* Secure, immutable system logging

> **Important:** This project does **not** implement the physical robot.
> It implements the **software architecture that controls, supervises, and safeguards modular robotic subsystems**.

---

## 2. Why This Project Matters (Problem Statement)

Modern assistive robots suffer from:

* Tight coupling between modules
* Fragile execution pipelines
* No formal intent lifecycle management
* Poor fault isolation
* No trustworthy audit trail

**ALTRUS Core solves this** by introducing:

* Intent‑driven execution (what the user wants, not how)
* Policy‑based routing
* Fault‑aware execution
* Automatic preemption and recovery
* Blockchain‑backed immutable logs
* Full CLI‑driven observability and control

---

## 3. High‑Level Architecture

```
┌───────────────┐
│ Intent Sources│  (CLI / ML / API / ROS)
└───────┬───────┘
        ↓
┌────────────────────┐
│  Intent Engine     │
│  • Validation      │
│  • Lifecycle Mgmt  │
│  • Preemption      │
└───────┬────────────┘
        ↓
┌────────────────────┐
│ Routing Policies   │
│ (YAML‑Driven)      │
└───────┬────────────┘
        ↓
┌────────────────────┐
│ Module Registry    │◄──── Health Monitor
│ • State            │
│ • Health           │
│ • Capabilities     │
└───────┬────────────┘
        ↓
┌────────────────────┐
│ Fault Engine       │
│ • Detection        │
│ • Recovery         │
│ • Escalation       │
└───────┬────────────┘
        ↓
┌────────────────────┐
│ Blockchain Ledger  │
│ (Immutable Logs)   │
└────────────────────┘
```

---

## 4. Implemented Core Components

### 4.1 Module Registry

The **Module Registry** is the single source of truth for all robot modules.

Each module has:

* `module_id`
* capabilities (e.g. `navigation.move`)
* lifecycle state
* health status
* heartbeat timestamp

#### Supported States

* `REGISTERED`
* `ACTIVE`
* `DEGRADED`
* `FAILED`
* `OFFLINE`

#### Supported Health Levels

* `HEALTHY`
* `UNSTABLE`
* `CRITICAL`
* `UNKNOWN`

#### Features

* Persistent storage (`~/.altrus/registry.json`)
* CLI‑controlled lifecycle
* Heartbeat‑based health evaluation
* Automatic fault escalation

---

### 4.2 Health Monitoring System

A background **Health Monitor** continuously evaluates module health based on:

* Last heartbeat timestamp
* Activation grace period
* Configurable thresholds

Health evaluation automatically:

* Updates module state
* Logs changes to the ledger
* Notifies the Intent Engine when a module fails

---

### 4.3 Fault Engine

The **Fault Engine** performs:

* Fault detection (heartbeat timeouts)
* Severity‑based escalation
* Recovery attempts
* System‑safe degradation

#### Current Recovery Strategy

* Restart module (mock strategy for demo)

> Recovery logic is intentionally extensible and pluggable.

---

### 4.4 Intent Engine (Core Contribution)

The **Intent Engine** is the heart of ALTRUS Core.

It manages the full **Intent Lifecycle**:

```
CREATED → VALIDATED → EXECUTING
           ↓
        PREEMPTED / REJECTED
```

#### Supported Features

* Intent validation
* Priority‑based arbitration
* Capability‑based routing
* Policy‑driven module selection
* Fault‑triggered preemption
* Persistent intent storage

#### Intent Sources

* CLI
* ML Adapter (simulated)
* HTTP / gRPC / ROS adapters (pluggable)

---

### 4.5 Policy‑Based Routing (YAML‑Driven)

Intent routing rules are defined declaratively using YAML:

* Preferred module health
* Allow degraded modules
* Deterministic selection
* Preemption thresholds

This enables **runtime behavior changes without code modification**.

---

### 4.6 Blockchain‑Backed Ledger

ALTRUS Core uses an **append‑only blockchain ledger** to record:

* Module state changes
* Health transitions
* Intent submission
* Intent execution
* Intent preemption

Each block contains:

* Hash
* Previous hash
* Timestamp
* Event payload

This ensures:

* Tamper‑resistant logs
* Full auditability
* Trustworthy demonstrations

---

### 4.7 Observability (Prometheus Metrics)

A Prometheus‑compatible metrics server runs on:

```
http://localhost:8000/metrics
```

#### Exposed Metrics

* Total modules
* Active modules
* Failed modules
* Total intents
* Executing intent indicator
* Fault & recovery counters

This enables:

* Real‑time system visibility
* Grafana integration (future work)

---

## 5. CLI‑Driven Demonstration Capabilities

All system behavior can be demonstrated **without writing any test code**.

### Module Operations

```bash
altrus modules add
altrus modules activate
altrus modules deactivate
altrus modules heartbeat
altrus modules list
```

### Intent Operations

```bash
altrus intents inject
altrus intents pending
altrus intents reconcile
altrus intents active
altrus intents list
```

### Ledger Inspection

```bash
altrus ledger
```

This CLI‑first design makes the system:

* Transparent
* Testable
* Demonstration‑ready

---

## 6. Demonstrated Scenarios (Completed)

✔ Module registration & activation
✔ Heartbeat‑based health monitoring
✔ Automatic module failure detection
✔ Intent injection via CLI & ML adapter
✔ Policy‑based intent routing
✔ Intent execution lifecycle
✔ Fault‑triggered intent preemption
✔ Persistent system state
✔ Immutable audit logging
✔ Prometheus metrics exposure

---

## 7. Current Limitations (Explicitly Acknowledged)

These are **intentionally left for Phase 2**:

* No physical robot integration
* No real hardware drivers
* No distributed event bus
* No UI / dashboard for ledger
* Recovery strategies are mocked

> These limitations are **design decisions**, not shortcomings.

---

## 8. Planned Work for Presentation 2 (April)

### 8.1 Advanced Intent Lifecycle

* Intent expiry (TTL enforcement)
* SLA violation handling
* Intent chaining

### 8.2 Event‑Driven Architecture

* Replace direct calls with Event Bus
* Asynchronous intent execution
* Cross‑component decoupling

### 8.3 Recovery & Resilience

* Multiple recovery strategies
* Intent rollback
* Safe‑mode execution

### 8.4 Visualization Layer

* Web‑based ledger viewer
* Grafana dashboards
* Intent timeline visualization

### 8.5 Integration Demonstration

* ROS / gRPC real adapters
* Multi‑module arbitration
* End‑to‑end robot simulation

---

## 9. Key Technical Contributions

* Fault‑aware intent execution model
* Policy‑driven routing engine
* Blockchain‑backed audit system
* CLI‑first middleware testing approach
* Language‑agnostic, platform‑agnostic design

---

## 10. Conclusion

ALTRUS Core demonstrates a **research‑grade middleware architecture** suitable for:

* Assistive robotics
* Safety‑critical systems
* Modular cyber‑physical platforms

The system is:

* Extensible
* Fault‑tolerant
* Auditable
* Demonstration‑ready

> This foundation enables confident expansion in Presentation 2 without architectural rework.

---

**Author:** Vihanga (VK)
**Project:** ALTRUS – Adaptive Life‑support & Therapeutic Robotic Unit System

