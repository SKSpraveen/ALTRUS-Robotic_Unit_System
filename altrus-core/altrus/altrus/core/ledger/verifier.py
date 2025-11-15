from .blockchain import BlockchainLedger


class LedgerVerifier:
    """
    Verifies integrity of the blockchain ledger.
    """

    @staticmethod
    def verify(ledger: BlockchainLedger) -> bool:
        chain = ledger.get_chain()

        for i in range(1, len(chain)):
            current = chain[i]
            previous = chain[i - 1]

            if current.previous_hash != previous.hash:
                return False

            if current.hash != current.compute_hash():
                return False

        return True

    @staticmethod
    def audit_trail(ledger: BlockchainLedger):
        report = []
        for block in ledger.get_chain():
            report.append({
                "index": block.index,
                "hash": block.hash,
                "previous": block.previous_hash,
                "event": block.data
            })
        return report