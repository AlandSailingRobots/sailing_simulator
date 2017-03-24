
from physics_models import WindState, PhysicsModel

class Simulation:
    def __init__( self, trueWind, timeModifier ):
        self._trueWind = trueWind
        self._timeModifier = timeModifier
        self._physicModels = []

    def addPhysicsModel( self, physicsModel ):
        self._physicModels.add( physicsModel )

    def step(self, timeDelta):
        finalTimeDelta = timeDelta * timeModifier

        for vessel in self._physicModels:
            vessel.simulate(finalTimeDelta)

        # Check for collisions


