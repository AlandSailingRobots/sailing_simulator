import numpy as np
from math import cos, sin, atan2, hypot
from utils import wrapTo2Pi, loadConfigFile
from class_wingsail import WingSail
import json


class WindState(object):
	def __init__(self, windDirection, windSpeed):
		self._dir = windDirection
		self._spd = windSpeed

	def direction(self):
		return self._dir

	def speed(self):
		return self._spd

class PhysicsModel:
	# X and y are in UTM coordinates, the heading is in radians
	def __init__(self, x = 0, y = 0, heading = 0):
		self._x = x
		self._y = y
		self._heading = heading
		self._rotationSpeed = 0
		self._speed = 0

	def utmCoordinate(self):
		return (self._x, self._y)

	def heading(self):
		return self._heading

	def speed(self):
		return self._speed

	def simulate(self, timeDelta, trueWind):
		raise NotImplementedError("Must override simulate method!")



class mainBoatPhysicsModel(PhysicsModel):
	def __init__(self, x = 0, y = 0, heading = 0, configPath = 'Janet_config.json' ):
		self._x             = x
		self._y             = y
		self._heading       = heading
		self._rotationSpeed = 0
		self._speed         = 0
		self._apparentWind  = WindState(0, 0)

	def apparentWind(self):
		return self._apparentWind	

	def updateApparentWind(self, trueWind ):
		apparentWindVector = [trueWind.speed() * cos( trueWind.direction() - self._heading ) - self._speed, trueWind.speed() * sin( trueWind.direction() - self._heading )]
		apparentWindAngle  = atan2(apparentWindVector[1], apparentWindVector[0])
		apparentWindSpeed  = hypot(apparentWindVector[0], apparentWindVector[1]) 
		self._apparentWind = WindState( apparentWindAngle, apparentWindSpeed )


		
	def forceOnRudder(self):
		return self._rudderLift * self._speed * sin( self._rudderAngle )



	def calculateDrift(self, trueWind ):
		x_boatVelocity = self._speed * cos(self._heading)
		y_boatVelocity = self._speed * sin(self._heading)

		x_windDrift    = self._driftCoefficient * trueWind.speed() * cos( trueWind.direction() )
		y_windDrift    = self._driftCoefficient * trueWind.speed() * sin( trueWind.direction() )

		x_dot          = x_boatVelocity + x_windDrift
		y_dot          = y_boatVelocity + y_windDrift

		return (x_dot, y_dot)









	

class SailingPhysicsModel(mainBoatPhysicsModel):
	# X and y are in UTM coordinates, the heading is in radians
	def __init__(self, x = 0, y = 0, heading = 0, configPath = 'Janet_config.json'):
		self._x                      = np.float64(x)
		self._y                      = np.float64(y)
		self._heading                = np.float64(heading)
		self._rotationSpeed          = np.float64(0)
		self._speed                  = 0

		config  = loadConfigFile(configPath)
		# sail parameters
		self._sailAngle              = 0
		self._sailLift               = config["sailLift"]              # kg s^-1

		#rudder parameters
		self._rudderAngle            = 0
		self._rudderLift             = config["rudderLift"]           # kg s^-1
		self._distanceToRudder       = config["distanceToRudder"]        # m
		self._rudderBreakCoefficient = config["rudderBreakCoefficient"]
		
		# wind parameters
		self._apparentWind           = WindState(0, 0)

		# parameters for dynamic
		self._driftCoefficient       = config["driftCoefficient"]
		self._tangentialFriction     = config["tangentialFriction"]    # kg s^-1
		self._angularFriction        = config["angularFriction"]  # kg m
		self._distanceToSailCoE      = config["distanceToSailCoE"]   # m
		self._distanceToMast         = config["distanceToMast"]   # m
		self._boatMass               = config["boatMass"]   # kg
		self._momentOfInertia        = config["momentOfInertia"]   # kg m^2
		

	def setActuators(self, sail, rudder):
		self._sailAngle = sail
		self._rudderAngle = rudder

	def getActuators(self):
		return ( self._sailAngle, self._rudderAngle )

	def apparentWind(self):
		return self._apparentWind


	def simulate(self, timeDelta, trueWind):
		(x_dot, y_dot) = self.calculateDrift( trueWind )
		self.updateApparentWind( trueWind )
		self.calculateCorrectSailAngle()

		# The acceleration of the boat is affected by three forces, the wind on the sail, a braking force
		# from the water on the rudder, and a tangential friction force
		sailForce = self.forceOnSails()
		rudderForce = self.forceOnRudder()

		sailAccelerationForce = sailForce * sin( self._sailAngle )
		rudderBrakeForce = self._rudderBreakCoefficient * rudderForce * sin( self._sailAngle )
		tangentialFictionForce = np.sign(self._speed) * (self._tangentialFriction * (self._speed)**2)

		acceleration_dot = ( (sailAccelerationForce - rudderBrakeForce) - tangentialFictionForce) / self._boatMass

		sailRotationForce = sailForce * ( self._distanceToSailCoE - self._distanceToMast * cos( self._sailAngle ) )
		rudderRotationForce = self._distanceToRudder * rudderForce * cos( self._rudderAngle )
		rotationForce = self._angularFriction * self._rotationSpeed * abs( self._speed )

		rotationSpeed_dot = (sailRotationForce - rudderRotationForce - rotationForce) / self._momentOfInertia
		
		self._x += x_dot * timeDelta
		self._y += y_dot * timeDelta
		self._heading += self._rotationSpeed * timeDelta
		self._speed += acceleration_dot * timeDelta
		self._rotationSpeed += rotationSpeed_dot * timeDelta
		self._heading = wrapTo2Pi(self._heading)


	# Ensures the sail is on the correct side of the boat
	def calculateCorrectSailAngle(self):
		sigma = cos( self._apparentWind.direction() ) + cos( self._sailAngle )
		if (sigma < 0):
			self._sailAngle = np.pi + self._apparentWind.direction()
		elif sin( self._apparentWind.direction() ) is not 0:
			# Ensure the sail is on the right side of the boat
			self._sailAngle = -np.sign( sin( self._apparentWind.direction() ) ) * abs( self._sailAngle )

	def forceOnSails(self):
		return self._sailLift * self._apparentWind.speed() * sin( self._sailAngle - self._apparentWind.direction() )


		
class ASPirePhysicsModel(mainBoatPhysicsModel):

	def __init__(self,x = 0, y = 0, heading = 0 , configPath = 'ASPire_config.json' , MWAngleStart = 0):
		self._x                      = np.float64(x)
		self._y                      = np.float64(y)
		self._heading                = np.float64(heading)
		self._rotationSpeed          = np.float64(0)
		self._speed                  = 0
		self._MWAngleStart           = MWAngleStart
		self._wingSail_config        = 'wingsail_config.json' # default

		config = loadConfigFile(configPath)
		
		# rudder parameters
		self._rudderAngle            = 0
		self._rudderLift             = config["rudderLift"]   # kg s^-1
		self._distanceToRudder       = config["distanceTorudder"]     # m
		self._rudderBreakCoefficient = config["rudderBreakCoefficient"] 
		# Wind parameters
		self._apparentWind           = WindState(0, 0)

		# parameters for dynamic
		self._driftCoefficient       = config["driftCoefficient"]
		self._tangentialFriction     = config["tangentialFriction"]    # kg s^-1
		self._angularFriction        = config["angularFriction"]  # kg m
		self._distanceToSailCoE      = config["distanceToSailCoE"]   # m
		self._distanceToMast         = config["distanceToMast"]   # m
		self._boatMass               = config["boatMass"]   # kg
		self._momentOfInertia        = config["momentOfInertia"]   # kg m^2

		if "wingSailConfigPath" in config.keys():
			self._wingSail_config = config["wingSailConfigPath"]
		# creation wingsail 
		self._wingSail               = WingSail(self._x,self._y,self._heading,self._MWAngleStart,0,self._distanceToSailCoE, self._wingSail_config)
		print('MWAngleStart:',self._MWAngleStart)


		
	def setActuators(self, tail, rudder):
		self._wingSail.setTailAngle(tail)
		self._rudderAngle = rudder

	def getActuators(self):
		return ( self._wingSail.getTailAngle(), self._rudderAngle )

	def apparentWind(self):
		return self._apparentWind

	def MWAngle(self):
		return self._wingSail.getMWAngle()
	 
	def simulate(self,timeDelta,trueWind):
		(x_dot, y_dot) = self.calculateDrift( trueWind )
		
		self.updateApparentWind( trueWind )

		# The acceleration of the boat is affected by three forces, the wind on the sail, a braking force
		# from the water on the rudder, and a tangential friction force
		wingSailForce                  = self._wingSail.rk2Step(self._apparentWind,timeDelta) 
		#Forces due to wind in the wind coordinate system
		rudderForce                    = self.forceOnRudder()
		#TO DO Check acceleration due to the main wing
		wingSailAccelerationForce      = wingSailForce * sin( self._wingSail.getMWAngle() )
		rudderBrakeForce               = self._rudderBreakCoefficient * rudderForce * sin( self._wingSail.getMWAngle() )
		tangentialFictionForce         = np.sign(self._speed) * (self._tangentialFriction * (self._speed)**2)
		speed_dot                      = ( (wingSailAccelerationForce - rudderBrakeForce) - tangentialFictionForce) / self._boatMass

		#TO DO check wingsail rotation force formula
		wingSailRotationForce          = wingSailForce * ( self._distanceToSailCoE - self._distanceToMast * cos( self._wingSail.getMWAngle() ) )
		rudderRotationForce            = self._distanceToRudder * rudderForce * cos( self._rudderAngle )
		rotationForce                  = self._angularFriction * self._rotationSpeed * abs( self._speed )

		rotationSpeed_dot              = (wingSailRotationForce - rudderRotationForce - rotationForce) / self._momentOfInertia

		self._x += x_dot * timeDelta
		self._y += y_dot * timeDelta
		self._heading += self._rotationSpeed * timeDelta
		self._speed += speed_dot * timeDelta
		self._rotationSpeed += rotationSpeed_dot * timeDelta
		self._heading = wrapTo2Pi(self._heading)
		
		
	



	
class SimplePhysicsModel(PhysicsModel):
	 # X and y are in UTM coordinates, the heading is in radians
	def __init__(self, heading = 0, speed = 0, x = 0, y = 0):
		self._x = x
		self._y = y
		self._heading = heading
		self._rotationSpeed = 0
		self._speed = speed

	def simulate(self, timeDelta, trueWind):
		velocityX = self._speed * cos(self._heading)
		velocityY = self._speed * sin(self._heading)
		self._x += velocityX * timeDelta
		self._y += velocityY * timeDelta



