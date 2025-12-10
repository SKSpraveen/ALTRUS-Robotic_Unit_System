import argparse
from altrus.cli.context import CLIContext
from altrus.core.kernel import AltrusKernel
from altrus.cli.commands import modules, intents, ledger


def main():
    kernel = AltrusKernel()
    ctx = CLIContext(kernel)

    parser = argparse.ArgumentParser(prog="altrus")
    sub = parser.add_subparsers(dest="command")

    modules.register(sub, ctx)
    intents.register(sub, ctx)
    ledger.register(sub, ctx)

    args = parser.parse_args()
    if hasattr(args, "func"):
        args.func(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
