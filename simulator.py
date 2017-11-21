
from physics_models import SailingPhysicsModel,mainBoatPhysicsModel, PhysicsModel, WindState, ASPirePhysicsModel


class Simulator:
    def __init__( self, trueWind, timeModifier ):
        self._trueWind = trueWind
        self._timeModifier = timeModifier
        self._physicModels = []

    def addPhysicsModel( self, physicsModel ):
        self._physicModels.append( physicsModel )

    def step(self, timeDelta):
        finalTimeDelta = timeDelta * self._timeModifier

        for vessel in self._physicModels:
            vessel.simulate( finalTimeDelta, self._trueWind )
