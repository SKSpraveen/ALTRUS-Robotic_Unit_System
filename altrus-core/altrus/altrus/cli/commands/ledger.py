def register(subparsers, ctx):
    parser = subparsers.add_parser("ledger", help="Ledger operations")
    parser.add_argument("--limit", type=int, default=20)
    parser.set_defaults(func=lambda args: tail_ledger(ctx, args.limit))


def tail_ledger(ctx, limit=20):
    print("\nLEDGER EVENTS:")
    chain = ctx.ledger.get_chain()[-limit:]

    for block in chain:
        event = block.data
        etype = event.get("event_type", "UNKNOWN")
        actor = f"{event.get('actor_type')}:{event.get('actor_id')}"
        print(f"[{block.index}] {etype} | {actor}")
