class ModuleNotFoundError(Exception):
    def __init__(self, module_id: str):
        super().__init__(f"Module '{module_id}' not found")
