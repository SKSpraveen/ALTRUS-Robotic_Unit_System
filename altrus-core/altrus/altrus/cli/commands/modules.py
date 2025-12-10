from altrus.core.registry.module import Module, ModuleState, ModuleHealth
import sys

def add_module(ctx, args):
    module = Module(
        module_id=args.id,
        name=args.name,
        version=args.version,
        capabilities=args.capabilities.split(","),
    )
    ctx.registry.register(module)
    print(f"Module '{args.id}' registered")


def remove_module(ctx, args):
    ctx.registry.unregister(args.id)
    print(f"Module '{args.id}' removed")


def list_modules(ctx):
    print("\nMODULES:")
    for m in ctx.registry.list_modules():
        print(
            f"{m.module_id:10} | "
            f"{m.state.value:9} | "
            f"{m.health.value:8} | "
            f"{','.join(m.capabilities)}"
        )

def activate_module(ctx, args):
    try:
        ctx.registry.activate(args.id)
        print(f"✅ Module '{args.id}' activated successfully")

    except Exception as e:
        print(f"❌ Failed to activate module '{args.id}': {e}")



def deactivate_module(ctx, args):
    try:
        module = ctx.registry.get(args.id)

        # Update state and health
        ctx.registry.update_state(args.id, ModuleState.OFFLINE)
        ctx.registry.update_health(args.id, ModuleHealth.UNKNOWN)

        # Save changes
        ctx.registry._storage.save(ctx.registry.list_modules())

        print(f"⏸️ Module '{args.id}' deactivated")
        print(f"   State: {module.state.value} → OFFLINE")
        print(f"   Health: {module.health.value} → UNKNOWN")
    except Exception as e:
        print(f"❌ Failed to deactivate module '{args.id}': {str(e)}", file=sys.stderr)
        sys.exit(1)



def register(subparsers, ctx):
    parser = subparsers.add_parser("modules", help="Module operations")
    sub = parser.add_subparsers(dest="action")

    # default: `altrus modules`
    parser.set_defaults(func=lambda _: list_modules(ctx))

    # list
    sub.add_parser("list").set_defaults(func=lambda _: list_modules(ctx))

    # add
    add = sub.add_parser("add", help="Register a module")
    add.add_argument("--id", required=True)
    add.add_argument("--name", required=True)
    add.add_argument("--version", default="1.0")
    add.add_argument("--capabilities", required=True)
    add.set_defaults(func=lambda args: add_module(ctx, args))

    # remove
    remove = sub.add_parser("remove", help="Unregister a module")
    remove.add_argument("--id", required=True)
    remove.set_defaults(func=lambda args: remove_module(ctx, args))

    #  activate command
    activate = sub.add_parser("activate", help="Activate a registered module")
    activate.add_argument("--id", required=True, help="Module ID to activate")
    activate.set_defaults(func=lambda args: activate_module(ctx, args))

    #  deactivate command while we're at it
    deactivate = sub.add_parser("deactivate", help="Deactivate a module")
    deactivate.add_argument("--id", required=True, help="Module ID to deactivate")
    deactivate.set_defaults(func=lambda args: deactivate_module(ctx, args))

    heartbeat = sub.add_parser("heartbeat", help="Send module heartbeat")
    heartbeat.add_argument("--id", required=True)
    heartbeat.set_defaults(func=lambda args: heartbeat_module(ctx, args))




def heartbeat_module(ctx, args):
    try:
        ctx.registry.heartbeat(args.id)
        ctx.ledger.log_event(
            event_type="MODULE_HEARTBEAT",
            actor_type="MODULE",
            actor_id=args.id,
            data={},
        )
        print(f"💓 Heartbeat sent for module '{args.id}'")
    except Exception as e:
        print(f"❌ {e}")
