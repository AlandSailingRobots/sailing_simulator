import numpy as np
from utils import wrapTo2Pi, trueWindCoordinatesToMWCoordinates, trueWindCoordinatesToBoatCoordinates

RHO = 1

class WingSail():
	def __init__(self,xboat = 0, yboat = 0,hboat = 0 , MWAngle = 0, tailAngle = 0, distanceToSailCOE = 0):
		self._MWAngle                          = np.float64(MWAngle)
		self._MWRotationSpeed                  = np.float64(0)
		self._tailAngle                        = np.float64(tailAngle)
		self._distanceToSailCoE                = distanceToSailCOE
		self._x                                = np.float64(xboat + distanceToSailCOE*np.cos(hboat))
		self._y                                = np.float64(yboat + distanceToSailCOE*np.sin(hboat))
		
		# Main Wing parameters
		self._MWChord                          = 0.74
		self._MWSpan                           = 2.785
		self._MWAspectRatio                    = self._MWChord/self._MWSpan
		self._MWArea                           = 0.1184*self._MWSpan
		self._MWConstPartWindForce             = (1/2)*RHO*self._MWArea
		self._MWDesignedLiftCoefficient        = 2*np.pi*self._MWAspectRatio/(self._MWAspectRatio+2)
		self._dontKnowHowToName                = (self._MWAspectRatio+1)/(self._MWAspectRatio+2)
		self._MWAngle                          = 0      # rad
		self._MWAngleDot                       = 0      # rad
		self._MWDistanceCOP                    = -0.10  # m
		
		# Tail parameters
		self._tailAngle                        = 0
		self._tailChord                        = 0.3    # m
		self._tailSpan                         = 0.8    # m
		self._tailAspectRatio                  = self._tailChord/self._tailSpan
		self._tailThickness                    = 0.055  # m
		self._tailArea                         = self._tailThickness*self._tailSpan
		self._tailDistanceToMast               = -0.812 #m
		self._tailConstPartWindForce           = 1/2*RHO*self._tailArea
		self._tailDesignedLiftCoefficient      = 2*np.pi*self._tailAspectRatio/(self._tailAspectRatio+2)
		self._tailMass                         = 2      # kg
		
		self._momentOfInertiaWingSailOnMast    = self._tailMass*self._tailDistanceToMast**2 
	
	
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
		return(wrapTo2Pi(apparentWind.direction()+self._MWAngle-np.pi))
		
	def forceOnMainWing(self,alpha):        
		liftForceMW = self._MWConstPartWindForce*self._MWDesignedLiftCoefficient*abs(wrapTo2Pi(alpha))*5.91
		dragForceMW = self._MWConstPartWindForce*self._MWDesignedLiftCoefficient*abs(wrapTo2Pi(alpha))**2*5.91/2
		if wrapTo2Pi(alpha) < 0:
			liftForceMW = -liftForceMW
		return (liftForceMW,dragForceMW)

	def forceOnTail(self,alpha):
		liftForceTail = self._tailConstPartWindForce*self._tailDesignedLiftCoefficient*abs(wrapTo2Pi(self._dontKnowHowToName*alpha-self._tailAngle))*5.91
		dragForceTail = self._tailConstPartWindForce*self._tailDesignedLiftCoefficient*abs(wrapTo2Pi(self._dontKnowHowToName*alpha-self._tailAngle))**2*5.91/2
		if wrapTo2Pi(alpha+self._tailAngle) < 0:
			liftForceTail  = -liftForceTail
		return  (liftForceTail, dragForceTail)
   
	def calculateMWAngleDotDot(self,MWLiftForce,MWDragForce,tailLiftForce,tailDragForce,alpha):
		xMW_TailRelatedPart,yMW_TailRelatedPart = trueWindCoordinatesToMWCoordinates(tailDragForce,tailLiftForce,alpha)
		tailRelatedMoment                       = yMW_TailRelatedPart*self._tailDistanceToMast
		tailRelatedPart                         = tailRelatedMoment/self._momentOfInertiaWingSailOnMast
		xMW_MWRelatedPart,yMW_MWRelatedPart     = trueWindCoordinatesToMWCoordinates(MWDragForce,MWLiftForce,alpha)
		MWRelatedMoment                         = yMW_MWRelatedPart*self._MWDistanceCOP
		MWRelatedPart                           = MWRelatedMoment/self._momentOfInertiaWingSailOnMast
		return(tailRelatedPart+MWRelatedPart)
	
	
	def evolutionWingSail(self,apparentWind):
		alpha                          = self.angleOfAttackMW(apparentWind)
		MWLiftForce,MWDragForce        = self.forceOnMainWing(alpha)
		tailLiftForce,tailDragForce    = self.forceOnTail(alpha)  
		MWAngleDotDot                  = self.calculateMWAngleDotDot(MWLiftForce,MWDragForce, tailLiftForce,tailDragForce,alpha)
		wingSailForceX,wingSailForceY  = trueWindCoordinatesToBoatCoordinates(MWDragForce,MWLiftForce,alpha,self._MWAngle)
		wingSailForce                  = np.sqrt(wingSailForceX**2+wingSailForceY**2)
		return(MWAngleDotDot,wingSailForce)
		
	def rk2Step(self, apparentWind,timeDelta):
		k1,wingSailForce      = self.evolutionWingSail(apparentWind)
		self._MWAngle         = self._MWAngle + timeDelta*self._MWRotationSpeed
		self._MWRotationSpeed = self._MWRotationSpeed + timeDelta*k1
		k2,useless            = self.evolutionWingSail(apparentWind)
		self._MWAngle         = self._MWAngle + timeDelta*self._MWRotationSpeed
		self._MWRotationSpeed = self._MWRotationSpeed + timeDelta*k2
		return(wingSailForce)
