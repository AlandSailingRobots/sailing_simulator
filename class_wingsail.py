import numpy as np
from utils import *

RHO = 1


class WingSail():

    def __init__(self,xboat = 0, yboat = 0,hboat = 0 , MWAngle = 0, tailAngle = 0, distanceToSailCOE = 0, configPath = 'wingsail_config.json'):
        self._MWAngle                          = wrapTo2Pi(np.float64(MWAngle))
        self._MWRotationSpeed                  = np.float64(0)
        self._tailAngle                        = np.float64(tailAngle)
        self._distanceToSailCoE                = distanceToSailCOE
        self._x                                = np.float64(xboat + distanceToSailCOE*np.cos(hboat))
        self._y                                = np.float64(yboat + distanceToSailCOE*np.sin(hboat))

        config = loadConfigFile(configPath)
        # Main Wing parameters
        self._MWChord                          = config["mainWingChord"] # m
        self._MWSpan                           = config["mainWingSpan"] # m
        self._MWAspectRatio                    = self._MWChord/self._MWSpan
        self._MWThickness                      = config["mainWingThickness"] # m
        self._MWArea                           = self._MWThickness*self._MWSpan
        self._MWConstPartWindForce             = (1/2)*RHO*self._MWArea
        self._MWDesignedLiftCoefficient        = 2*np.pi*self._MWAspectRatio/(self._MWAspectRatio+2)
        self._dontKnowHowToName                = (self._MWAspectRatio+1)/(self._MWAspectRatio+2)        
        self._MWAngleDot                       = 0      # rad
        self._MWDistanceCOP                    = config["distanceCenterOfPressureToMast"] # m
        
        
        # Tail parameters
        self._tailAngle                        = 0
        self._tailChord                        = config["tailChord"]    # m
        self._tailSpan                         = config["tailSpan"]    # m
        self._tailAspectRatio                  = self._tailChord/self._tailSpan
        self._tailThickness                    = config["tailThickness"]  # m
        self._tailArea                         = self._tailThickness*self._tailSpan
        self._tailDistanceToMast               = config["tailDistanceToMast"] #m
        self._tailConstPartWindForce           = 1/2*RHO*self._tailArea
        self._tailDesignedLiftCoefficient      = 2*np.pi*self._tailAspectRatio/(self._tailAspectRatio+2)
        self._tailMass                         = config["tailMass"]      # kg
        
        self._momentOfInertiaWingSailOnMast    = (self._tailMass*self._tailDistanceToMast**2 + 11*self._MWDistanceCOP**2) # added mass main wing
        
        print('MWAngleWingsailClass:',self._MWAngle)

    def getPosition(self):
        return(self._x,self._y)
    
    def setPosition(self,xboat,yboat,hboat):
        self._x = np.float64(xboat + self._distanceToSailCoE*np.cos(hboat))
        self._y = np.float64(yboat + self._distanceToSailCOE*np.Sin(hboat))

        
        
    def getMWAngle(self):
        return(self._MWAngle)
    
    def setTailAngle(self,tail):
        self._tailAngle = tail
    
    def getTailAngle(self):
        return(self._tailAngle)
    
    def angleOfAttackMW(self,apparentWind):
        #print('MWA:',self._MWAngle)
        #print('ap_wind:',apparentWind.direction())
        return(wrapTo2Pi(apparentWind.direction()-self._MWAngle-np.pi))
        
    def forceOnMainWing(self,alpha,apparentWind):        
        liftForceMW = self._MWConstPartWindForce*(apparentWind.speed()**2)*self._MWDesignedLiftCoefficient*abs(wrapTo2Pi(alpha))*5.91
        dragForceMW = self._MWConstPartWindForce*(apparentWind.speed()**2)*self._MWDesignedLiftCoefficient*abs(wrapTo2Pi(alpha))**2*5.91/2
        if wrapTo2Pi(alpha) > 0:
            liftForceMW = -liftForceMW
        return (liftForceMW,dragForceMW)

    def forceOnTail(self,alpha, apparentWind):
        liftForceTail = self._tailConstPartWindForce*(apparentWind.speed()**2)*self._tailDesignedLiftCoefficient*abs(wrapTo2Pi(self._dontKnowHowToName*alpha-self._tailAngle))*5.91
        dragForceTail = self._tailConstPartWindForce*(apparentWind.speed()**2)*self._tailDesignedLiftCoefficient*abs(wrapTo2Pi(self._dontKnowHowToName*alpha-self._tailAngle))**2*5.91/2
        if wrapTo2Pi(alpha-self._tailAngle) > 0:
            liftForceTail  = -liftForceTail
        return  (liftForceTail, dragForceTail)
   
    def calculateMWAngleDotDot(self,MWLiftForce,MWDragForce,tailLiftForce,tailDragForce,alpha):
        xMW_TailRelatedPart,yMW_TailRelatedPart = apparentWindCoordinatesToMWCoordinates(tailDragForce,tailLiftForce,alpha)
        tailRelatedMoment                       = yMW_TailRelatedPart*self._tailDistanceToMast
        tailRelatedPart                         = tailRelatedMoment/self._momentOfInertiaWingSailOnMast
        xMW_MWRelatedPart,yMW_MWRelatedPart     = apparentWindCoordinatesToMWCoordinates(MWDragForce,MWLiftForce,alpha)
        MWRelatedMoment                         = yMW_MWRelatedPart*self._MWDistanceCOP
        MWRelatedPart                           = MWRelatedMoment/self._momentOfInertiaWingSailOnMast
        return(tailRelatedPart+MWRelatedPart) 
    
    
    def evolutionWingSail(self,apparentWind):
        alpha                          = self.angleOfAttackMW(apparentWind)
        #print('alpha',alpha)
        MWLiftForce,MWDragForce        = self.forceOnMainWing(alpha, apparentWind)
        tailLiftForce,tailDragForce    = self.forceOnTail(alpha, apparentWind)  
        MWAngleDotDot                  = self.calculateMWAngleDotDot(MWLiftForce,MWDragForce, tailLiftForce,tailDragForce,alpha)
        wingSailForceX,wingSailForceY  = apparentWindCoordinatesToBoatCoordinates(MWDragForce,MWLiftForce,alpha,self._MWAngle)
        wingSailForce                  = np.sqrt(wingSailForceX**2+wingSailForceY**2)
        return(MWAngleDotDot,wingSailForce)
        
    def rk2Step(self, apparentWind,timeDelta):
        #print('timeDelta', timeDelta)
        k1,wingSailForce      = self.evolutionWingSail(apparentWind)
        self._MWAngle         = wrapTo2Pi(self._MWAngle + timeDelta*self._MWRotationSpeed)
        self._MWRotationSpeed = wrapTo2Pi(self._MWRotationSpeed + timeDelta*k1)
        #print('K1 MWAngle:', self._MWAngle, 'K1 Speed:', self._MWRotationSpeed)
        k2,wingSailForce      = self.evolutionWingSail(apparentWind)
        self._MWAngle         = wrapTo2Pi(self._MWAngle + timeDelta*self._MWRotationSpeed)
        self._MWRotationSpeed = wrapTo2Pi(self._MWRotationSpeed + timeDelta*k2)
        #print('K2 MWAngle:', self._MWAngle, 'K2 Speed:', self._MWRotationSpeed)
        return(wingSailForce)


    def wingSailFor(self, apparentWind,timeDelta):
        mwangledd,wingSailForce     = self.evolutionWingSail(apparentWind)
        self._MWAngle               = wrapTo2Pi(self._MWAngle + timeDelta*self._MWRotationSpeed)
        self._MWRotationSpeed       = wrapTo2Pi(self._MWRotationSpeed + timeDelta*mwangledd)

        return (wingSailForce)
