class CLIContext:
    def __init__(self, kernel):
        self.kernel = kernel
        self.registry = kernel.registry
        self.intent_engine = kernel.intent_engine
        self.intent_definitions = kernel.intent_definitions
        self.ledger = kernel.ledger
