from abc import ABC, abstractmethod


class AnalysisMonitor(ABC):
    def __init__(self, name: str):
        if not isinstance(name, str) or not name:
            raise ValueError("Monitor name must be a non-empty string.")
        self.name = name

    def attach(self, sat):
        pass

    @abstractmethod
    def sample(self, sat):
        pass
