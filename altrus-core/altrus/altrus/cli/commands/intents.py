import sys
import uuid
from altrus.core.intent.intent import Intent
from altrus.core.intent.intent import IntentPriority
from altrus.core.intent.intent import IntentState


def inject_intent(ctx, args):
    try:
        definition = ctx.intent_definitions.get(args.name)
        if not definition:
            raise ValueError(f"Unknown intent: {args.name}")

        intent = Intent(
            intent_id=str(uuid.uuid4()),
            name=definition.name,
            capability=definition.route["capability"],
            priority=IntentPriority(definition.priority),
            preemptible=definition.preemptible,
            ttl=definition.ttl,
        )

        ctx.intent_engine.submit_intent(intent)
        print(f"✅ Intent injected: {intent.name} ({intent.intent_id})")

    except Exception as e:
        print(f"❌ Failed to inject intent: {e}", file=sys.stderr)
        sys.exit(1)


def list_intents(ctx):
    intents = ctx.intent_engine.list_intents()

    print("\nINTENTS:")
    for i in intents:
        print(
            f"{i.name:16} | "
            f"{i.state.value:10} | "
            f"{i.priority.name:8} | "
            f"{i.capability}"
        )


def reconcile_intents(ctx):
    """
    Re-evaluate pending intents against current system state.
    """
    ctx.intent_engine.execute_pending_intents()
    print("🔁 Intent reconciliation triggered")


def register(subparsers, ctx):
    parser = subparsers.add_parser("intents", help="Intent operations")
    sub = parser.add_subparsers(dest="action")

    # default: list
    parser.set_defaults(func=lambda _: list_intents(ctx))

    # list
    sub.add_parser("list").set_defaults(func=lambda _: list_intents(ctx))

    # inject (NAMED INTENT)
    inject = sub.add_parser("inject", help="Inject a named intent")
    inject.add_argument("--name", required=True, help="Intent name (e.g. NAVIGATE)")
    inject.set_defaults(func=lambda args: inject_intent(ctx, args))

    reconcile = sub.add_parser("reconcile", help="Reconcile pending intents")
    reconcile.set_defaults(func=lambda _: reconcile_intents(ctx))

    force = sub.add_parser("force", help="Force intent execution (demo mode)")
    force.set_defaults(func=lambda _: force_execute(ctx))

    # pending
    sub.add_parser("pending").set_defaults(
        func=lambda _: list_pending_intents(ctx)
    )

    # active
    sub.add_parser("active").set_defaults(
        func=lambda _: list_active_intent(ctx)
    )



def force_execute(ctx):
    intents = ctx.intent_engine.intent_store

    # Pick highest-priority VALIDATED intent
    candidates = [
        i for i in intents.values()
        if i.state == IntentState.VALIDATED
    ]

    if not candidates:
        print("No pending intents")
        return

    candidates.sort(key=lambda i: i.priority, reverse=True)
    intent = candidates[0]

    # 🔥 AUTHORITATIVE STATE CHANGE
    intent.state = IntentState.EXECUTING
    ctx.intent_engine.current_intent = intent

    # Save ALL intents back
    ctx.intent_engine.storage.save(intents.values())

    ctx.ledger.log_event(
        event_type="INTENT_EXECUTING",
        actor_type="INTENT",
        actor_id=intent.intent_id,
        data={"forced": True},
    )

    print(f"🔥 Intent EXECUTING: {intent.name}")

def list_pending_intents(ctx):
    intents = ctx.intent_engine.list_intents()

    print("\nPENDING INTENTS:")
    for i in intents:
        if i.state.value == "VALIDATED":
            print(
                f"{i.name:16} | "
                f"{i.priority.name:8} | "
                f"{i.capability}"
            )

def list_active_intent(ctx):
    intents = ctx.intent_engine.list_intents()

    active = [
        i for i in intents
        if i.state == IntentState.EXECUTING
    ]

    print("\nACTIVE INTENT:")
    if not active:
        print("None")
        return

    intent = active[0]
    print(
        f"{intent.name:16} | "
        f"{intent.priority.name:8} | "
        f"{intent.capability} | "
        f"EXECUTING"
    )

