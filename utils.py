# -*- coding: utf-8 -*-
"""
Created on Wed Jun 21 11:02:38 2017

@author: jazicoan
"""
import numpy as np




	# ===== Angle-related functions ===== #

def degTorad(angle):
	return((angle*np.pi/180))

	
def radTodeg(angle):
	return(angle*180/np.pi)

def wrapTo2Pi(theta):
	theta = 2.0*np.arctan(np.tan(theta/2.0))
	return(float(theta))

def wrapAngle(theta):
	if theta < 0:
		theta = theta+360
	if theta > 360:
		theta = theta%360
	return(theta) 


	# Add new functions or sections if necessary

    # ===== Switch between Coordinate System ===== #

def trueWindCoordinatesToMWCoordinates(FxW,FyW,alpha):
	Fy = FyW*np.cos(alpha)-FxW*np.sin(alpha)
	Fx = FyW*np.sin(alpha)+FxW*np.cos(alpha)
	return (Fx,Fy)

def MWCoordinatesToBoatCoordinates(FxMW,FyMW,MWAngle):
	Fx = FxMW*np.cos(MWAngle)+FyMW*np.sin(MWAngle)
	Fy = FyMW*np.cos(MWAngle)-FxMW*np.sin(MWAngle)
	return (Fx,Fy)

def trueWindCoordinatesToBoatCoordinates(FxW,FyW,alpha,MWAngle):
	FxMW,FyMW = trueWindCoordinatesToMWCoordinates(FxW,FyW,alpha)
	return(MWCoordinatesToBoatCoordinates(FxMW,FyMW,MWAngle))


    # ===== Integration Schemes ===== #

def euler(x,xdot,dt):
	return(x+xdot*dt)


	


    # Add new functions or sections if necessary
