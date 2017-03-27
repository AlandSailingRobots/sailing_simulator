
from physics_models import PhysicsModel, WindState


class Simulator:
    def __init__(self, vessels, trueWind):
        self._vessels = vessels
        self._trueWind = trueWind

    def step(self):
        # TODO, variable time step
        timeStep = 0.05

        for vessel in self._vessels:
            physics = vessel.physicsModel()
            physics.simulate( timeStep, self._trueWind )

        # Check for collisions
