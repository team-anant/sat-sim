class Component:
    def __init__(self, name: str):
        if not isinstance(name, str) or not name:
            raise ValueError("Component name must be a non-empty string.")
        self.name = name

    def attach(self, sat):
        pass
