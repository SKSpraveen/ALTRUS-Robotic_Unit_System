
from altrus.core.kernel import AltrusKernel
from altrus.core.registry.module import Module

kernel = AltrusKernel()

kernel.registry.register(
    Module(
        module_id="nav01",
        name="Navigation",
        version="1.0",
        capabilities=["navigation.move"]
    )
)

kernel.registry.register(
    Module(
        module_id="tele01",
        name="Telemedicine",
        version="1.0",
        capabilities=["telemedicine.call"]
    )
)

print(kernel.registry.list_modules())

