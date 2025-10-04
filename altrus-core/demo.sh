#!/usr/bin/env bash
set -e

echo "==============================="
echo "ALTRUS END-TO-END DEMO"
echo "==============================="

echo ""
echo "🧹 1) Clean previous state"
rm -rf ~/.altrus
sleep 1

echo ""
echo "📦 2) Register module (Navigation)"
altrus modules add \
  --id nav1 \
  --name NavigationModule \
  --capabilities navigation.move

echo ""
echo "🚀 3) Activate module"
altrus modules activate --id nav1
sleep 1
altrus modules

echo ""
echo "📨 4) Inject intent (NAVIGATE)"
altrus intents inject --name NAVIGATE
sleep 1

echo ""
echo "📋 5) Pending intents"
altrus intents pending

echo ""
echo "🔁 6) Reconcile intents → EXECUTING"
altrus intents reconcile
sleep 1
altrus intents active

echo ""
echo "📊 7) Ledger after execution"
altrus ledger

echo ""
echo "💓 8) Send heartbeat (module stays healthy)"
altrus modules heartbeat --id nav1
sleep 2
altrus modules

echo ""
echo "⏳ 9) Simulate failure (no heartbeat)"
echo "   Waiting 40 seconds..."
sleep 40

echo ""
echo "💥 10) Module failed → Intent preempted"
altrus modules
altrus intents active
altrus intents list

echo ""
echo "📜 11) Ledger after fault & preemption"
altrus ledger

echo ""
echo "🔄 12) Recover module"
altrus modules activate --id nav1
sleep 1
altrus modules

echo ""
echo "📨 13) Inject intent again"
altrus intents inject --name NAVIGATE
sleep 1
altrus intents reconcile
sleep 1
altrus intents active

echo ""
echo "📈 14) Observability (Prometheus metrics)"
echo "Open in browser:"
echo "👉 http://localhost:8000/metrics"

echo ""
echo "✅ DEMO COMPLETE"

echo "python ./examples/demo_run.py"