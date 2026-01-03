# ALTRUS – Developer Documentation

> **ALTRUS (Adaptive Life-support & Therapeutic Robotic Unit System)**
>
> Developer-focused documentation for understanding, extending, and contributing to the ALTRUS middleware core.

---

## 1. Purpose of This Document

This document is written **for developers and evaluators**, not end-users.

It explains:

* Internal architecture and design decisions
* How core subsystems interact
* How to extend the system safely
* What is intentionally incomplete (future scope)

This documentation is suitable for **GitHub Pages publication**.

---

## 2. System Overview

ALTRUS is a **fault-tolerant middleware architecture** designed for **modular assistive robots**.

It acts as the **authoritative control layer** between:

* Intent sources (CLI, ML models, APIs)
* Robot capability modules (navigation, telemedicine, safety)

The system is:

* Event-driven
* State-authoritative
* Fault-aware
* Auditable
* Observable

---

## 3. High-Level Architecture

```
Intent Sources
 (CLI / ML / API)
        │
        ▼
┌────────────────────┐
│   Intent Engine    │◄──── Policy Engine
└────────────────────┘
        │
        ▼
┌────────────────────┐
│  Module Registry   │◄──── Health Monitor
└────────────────────┘
        │
        ▼
┌────────────────────┐
│   Fault Engine     │◄──── Detectors
└────────────────────┘
        │
        ▼
┌────────────────────┐
│ Blockchain Ledger  │
└────────────────────┘
        │
        ▼
┌────────────────────┐
│ Observability      │ (Prometheus)
└────────────────────┘
```

---

## 4. Core Subsystems (Deep Dive)

### 4.1 Module Registry

**Responsibility**: System ground truth for all robot modules.

#### Key Concepts

* `ModuleState`: ACTIVE, OFFLINE, FAILED
* `ModuleHealth`: HEALTHY, DEGRADED, CRITICAL
* Capability-based lookup

#### Responsibilities

* Register/unregister modules
* Track activation state
* Track heartbeat timestamps
* Evaluate health via `HealthEvaluator`
* Notify Intent Engine on failures

#### Design Choice

The registry is **authoritative**. No other subsystem mutates module state directly.

---

### 4.2 Health Monitor

**Responsibility**: Continuous evaluation of module health.

#### How It Works

* Runs on a background thread
* Periodically calls `ModuleRegistry.evaluate_health()`
* Detects heartbeat timeouts
* Transitions modules to FAILED state

#### Why This Matters

* Enables proactive fault detection
* Decouples health logic from business logic

---

### 4.3 Fault Engine

**Responsibility**: Fault detection, classification, and recovery.

#### Components

* Detectors (e.g., `HeartbeatDetector`)
* Recovery strategies (e.g., `RestartModuleStrategy`)

#### Current Behavior

* Detects module failure
* Notifies Intent Engine
* Attempts basic recovery (mock strategy)

> ⚠️ Recovery strategies are **simulated** in this phase.

---

### 4.4 Intent Engine (Core Contribution)

**Responsibility**: Authoritative intent lifecycle management.

#### Intent Lifecycle

```
CREATED → VALIDATED → EXECUTING
                     ↓
                 PREEMPTED / REJECTED
```

#### Responsibilities

* Intent validation
* Priority-based selection
* Capability-based routing
* Preemption handling
* Fault-aware execution

#### Key Design Principles

* Only **one intent may EXECUTE at a time**
* Intent state transitions are authoritative
* Execution depends on module health + capability

---

### 4.5 Routing Policy Engine

**Responsibility**: Decide *which module* should execute an intent.

#### Current Policy

* YAML-driven configuration
* Health-preferred routing
* Deterministic module selection

#### Why Policy Is Pluggable

Allows future research on:

* Learning-based routing
* Load-aware routing
* SLA-based routing

---

### 4.6 Blockchain Ledger

**Responsibility**: Immutable audit trail of system events.

#### Logged Events

* Module registration
* State changes
* Health changes
* Intent submission
* Intent execution
* Intent preemption

#### Why Blockchain (Even Locally)

* Tamper-evident logs
* Research relevance
* Compliance & audit use-cases

---

### 4.7 Observability (Prometheus)

**Responsibility**: Runtime system visibility.

#### Metrics Exposed

* Total modules
* Active modules
* Failed modules
* Total intents
* Intent execution state

#### Access

```
http://localhost:8000/metrics
```

#### Design Decision

* Metrics are **read-only**
* No control logic depends on metrics

---

## 5. Adapters (Integration Layer)

Adapters allow **external systems** to interact with ALTRUS without coupling.

### Implemented Adapters

* CLI Adapter (primary demo interface)
* ML Adapter (simulated intent injection)

### Planned Adapters

* HTTP Adapter (REST)
* gRPC Adapter
* ROS Adapter

> Adapters only **translate inputs**. They never own logic.

---

## 6. CLI Design Philosophy

The CLI is designed as:

* A developer tool
* A testing harness
* A demo interface

All CLI commands:

* Call core APIs
* Never mutate state directly
* Log events via ledger

---

## 7. Persistence Model

Persistent state stored under:

```
~/.altrus/
```

Includes:

* Registered modules
* Intent history
* Ledger chain

This allows:

* CLI restarts without losing state
* Crash-safe recovery

---

## 8. Known Limitations (Intentional)

The following are **deliberately excluded** from this phase:

* Real robot execution
* Networked modules
* Real fault recovery
* Distributed consensus
* Web UI / dashboards

This avoids **scope creep** and preserves research focus.

---

## 9. Future Extensions (Presentation 2 Scope)

### 9.1 Real-Time Event Bus

* Replace in-process bus
* Enable distributed messaging

### 9.2 Plugin System

* Dynamic module loading
* Hot-swappable capabilities

### 9.3 Smart Recovery Engine

* ML-based fault classification
* Adaptive recovery strategies

### 9.4 Ledger Web Dashboard

* Web UI for blockchain events
* Timeline visualization

### 9.5 Grafana Integration

* Prometheus → Grafana dashboards
* Long-term system analytics

---

## 10. Contribution Guidelines

For contributors:

1. Never mutate state outside core engines
2. Log all significant events
3. Preserve deterministic behavior
4. Avoid coupling adapters to core logic

---

## 11. Final Notes

ALTRUS is designed as a **research-grade middleware**, not a product.

Its value lies in:

* Architectural clarity
* Fault-aware design
* Extensibility
* Clear separation of concerns

This foundation is intentionally strong to support future innovation.

---

© ALTRUS Project – Developer Documentation
