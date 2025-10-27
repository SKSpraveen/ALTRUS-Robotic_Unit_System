from abc import ABC, abstractmethod


class RoutingPolicy(ABC):
    @abstractmethod
    def select_module(self, intent, candidate_modules):
        """
        Returns a selected module or None
        """
        pass
