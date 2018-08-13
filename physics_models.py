import numpy as np
from math import cos, sin, atan2, hypot
from utils import wrapTo2Pi, loadConfigFile
import json


class WindState(object):
    def __init__(self, windDirection, windSpeed):
        self._dir = windDirection # radian, east north up
        self._spd = windSpeed # m/s

    def direction(self):
        return self._dir

    def speed(self):
        return self._spd




class PhysicsModel:
    # X and y are in UTM coordinates, the heading is in radians
    def __init__(self, x=0, y=0, heading=0):
        self._x = x
        self._y = y
        self._heading = heading # [-pi, pi] east north up
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
        self._sailLift               = config["sailLift"]                # kg s^-1

        #rudder parameters
        self._rudderAngle            = 0
        self._rudderLift             = config["rudderLift"]              # kg s^-1
        self._distanceToRudder       = config["distanceToRudder"]        # m
        self._rudderBreakCoefficient = config["rudderBreakCoefficient"]
        
        # wind parameters
        self._apparentWind           = WindState(0, 0)

        # parameters for dynamic
        self._driftCoefficient       = config["driftCoefficient"]
        self._tangentialFriction     = config["tangentialFriction"]      # kg s^-1
        self._angularFriction        = config["angularFriction"]         # kg m
        self._distanceToSailCoE      = config["distanceToSailCoE"]       # m
        self._distanceToMast         = config["distanceToMast"]          # m
        self._boatMass               = config["boatMass"]                # kg
        self._momentOfInertia        = config["momentOfInertia"]         # kg m^2
        

    def setActuators(self, sail, rudder):
        self._sailAngle = sail
        self._rudderAngle = rudder

    def getActuators(self):
        return ( self._sailAngle, self._rudderAngle )


    def simulate(self, timeDelta, trueWind):
        (x_dot, y_dot) = self.calculateDrift( trueWind )
        self.updateApparentWind( trueWind )
        self.calculateCorrectSailAngle()

        # The acceleration of the boat is affected by three forces, the wind on the sail, a braking force
        # from the water on the rudder, and a tangential friction force
        sailForce = self.forceOnSails()
        rudderForce = self.forceOnRudder()

        sailAccelerationForce = sailForce * sin(self._sailAngle)

        rudderBrakeForce = self._rudderBreakCoefficient * rudderForce * sin(self._rudderAngle)

        tangentialFictionForce = np.sign(self._speed) * (self._tangentialFriction * (self._speed)**2)

        acceleration_dot = ( (sailAccelerationForce - rudderBrakeForce) - tangentialFictionForce) / self._boatMass

        sailRotationForce = sailForce * ( self._distanceToSailCoE - self._distanceToMast * cos(self._sailAngle ))
        rudderRotationForce = self._distanceToRudder * rudderForce * cos(self._rudderAngle)
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
        if sin( self._apparentWind.direction() ) == 0:
            self._sailAngle = min( abs(wrapTo2Pi(np.pi - self._apparentWind.direction())), abs(wrapTo2Pi(self._sailAngle)) )
        else:
            self._sailAngle = - np.sign( sin( self._apparentWind.direction() ) ) * min( abs(wrapTo2Pi(np.pi - self._apparentWind.direction())), abs(wrapTo2Pi(self._sailAngle)) )

    def forceOnSails(self):
        return self._sailLift * self._apparentWind.speed() * sin( self._sailAngle - self._apparentWind.direction() )




# ASPire model has been modified, apparent wind is calculated in world coord., wingsail angle is set manually depending on tail angle. Only MW lift and drag considered, not tail.
# MWAngle and apparentWind kept so as not to need changes in other parts of simulator.

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
        self._apparentWind           = WindState(0, 0)  # apparent wind in boat coord
        self._apparentWindGlobalCoord        = WindState(0, 0)  # apparent wind in world coord.

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
        


        self._tailAngle              = 0    # angle of tail with regards to main wing center line, + to the right, rad
        self._MWAngleG               = 0    # angle of main wing in with regards to world coord. syst., east=0 north=pi/2
        self._MWAngleB               = 0    # angle of main wing with regards to boat centerline, + to the right
        self._alpha                  = 0.30    # chosen angle of attack for simple simulation

        # wingsail parameters:

        RHO                          = 1   # density of air

        config = loadConfigFile('wingsail_config.json')
       
        # Main Wing parameters
        self._MWChord                          = config["mainWingChord"] # m
        self._MWSpan                           = config["mainWingSpan"] # m
        self._MWAspectRatio                    = self._MWChord/self._MWSpan
        self._MWThickness                      = config["mainWingThickness"] # m
        self._MWArea                           = self._MWThickness*self._MWSpan
        self._MWConstPartWindForce             = (1/2)*RHO*self._MWArea
        self._MWDesignedLiftCoefficient        = 2*np.pi*self._MWAspectRatio/(self._MWAspectRatio+2)
        self._dontKnowHowToName                = (self._MWAspectRatio+1)/(self._MWAspectRatio+2)        
       
   
        
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
        
        


        
    def setActuators(self, tail, rudder):
        self._tailAngle = tail
        self._rudderAngle = rudder

    def getActuators(self):
        return ( self._tailAngle, self._rudderAngle )

        # updates apparent wind ASP (aka apparent wind in world coord)
    def updateApparentWindGlobalCoord(self, trueWind ):
        
        Xaw                = trueWind.speed()*cos(trueWind.direction()) - self.speed()*cos(self.heading())
        Yaw                = trueWind.speed()*sin(trueWind.direction()) - self.speed()*sin(self.heading())


        apparentWindVector = [Xaw,Yaw]
        apparentWindSpeed  = np.sqrt(Xaw**2 + Yaw**2) 


        if Xaw > 0 :                                                        #necessary since arctan maps to -pi/2 ; pi/2
            apparentWindAngle  = wrapTo2Pi(np.arctan(Yaw/Xaw))

        elif Xaw < 0 :
            apparentWindAngle  = wrapTo2Pi(np.arctan(Yaw/Xaw) + np.pi)

        elif Xaw == 0 :
            if Yaw < 0 :
                apparentWindAngle  = -np.pi/2
            elif Yaw > 0 :
                apparentWindAngle  = np.pi/2

        self._apparentWindGlobalCoord = WindState( apparentWindAngle, apparentWindSpeed )

    
    def apparentWindGlobalCoord(self):
        return self._apparentWindGlobalCoord()


    def MWAngle(self):
        return self._MWAngleB
     
    def simulate(self,timeDelta,trueWind):
        (x_dot, y_dot) = self.calculateDrift( trueWind )
        
        self.updateApparentWind( trueWind )

        self.updateApparentWindGlobalCoord( trueWind )

   
        liftForceMW = self._MWConstPartWindForce*(self._apparentWindGlobalCoord.speed()**2)*self._MWDesignedLiftCoefficient*abs(wrapTo2Pi(self._alpha))*5.91*10
        dragForceMW = self._MWConstPartWindForce*(self._apparentWindGlobalCoord.speed()**2)*self._MWDesignedLiftCoefficient*abs(wrapTo2Pi(self._alpha))**2*10*5.91/2


        MWAngleGint = wrapTo2Pi(self._apparentWindGlobalCoord.direction()+np.pi)  # initially set wingsail main wing facing into the wind, then set angle of attack depending on tail

        if self._tailAngle < 0 :
            self._MWAngleG = wrapTo2Pi(MWAngleGint  + self._alpha)  # lift force is -90dg from drag force

            
        elif self._tailAngle > 0 :
            self._MWAngleG = wrapTo2Pi(MWAngleGint  - self._alpha) # lift force is 90dg from drag force
            liftForceMW = -liftForceMW

        else :
            self._MWAngleG = MWAngleGint
            liftForceMW = 0




        # angle of wingsail with regards to boat
        self._MWAngleB = wrapTo2Pi(self._MWAngleG -self._heading)


        # lift and drag of main wing projected onto x axis of boat (in direction of movement forward)
        wingSailForce = -dragForceMW * cos(self._apparentWindGlobalCoord.direction()-self.heading()) + liftForceMW * (sin(self._apparentWindGlobalCoord.direction()-self.heading()))

        print('wignsailforce',wingSailForce)
        #print('liftforce',liftForceMW)
        #print('dragForceMW',dragForceMW)
        #print('app wind direc',self._apparentWindGlobalCoord.direction())
        #print('app wind speed',self._apparentWindGlobalCoord.speed())


    
        rudderForce                    = self.forceOnRudder()

       
      
        rudderBrakeForce               = np.sign(self._speed) * self._rudderBreakCoefficient * rudderForce * np.abs(sin( wrapTo2Pi(self._rudderAngle) )) # always against speed
       
        tangentialFictionForce         = np.sign(self._speed) * self._tangentialFriction * (self._speed)**2   # always agaisnt speed
      
        speed_dot                      = ( wingSailForce - rudderBrakeForce - tangentialFictionForce) / self._boatMass

        


        rudderRotationForce            = self._distanceToRudder * rudderForce * cos( wrapTo2Pi(self._rudderAngle) )
        rotationForce                  = self._angularFriction * self._rotationSpeed * abs( self._speed )

        rotationSpeed_dot              = (- rudderRotationForce - rotationForce) / self._momentOfInertia

        self._x += x_dot * timeDelta
        self._y += y_dot * timeDelta
        self._heading += self._rotationSpeed * timeDelta
        self._speed += speed_dot * timeDelta
        self._rotationSpeed += rotationSpeed_dot * timeDelta
        self._heading = wrapTo2Pi(self._heading)

    





class SimplePhysicsModel(PhysicsModel):
    # X and y are in UTM coordinates, the heading is in radians
    def __init__(self, heading=0, speed=0, x=0, y=0):
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

