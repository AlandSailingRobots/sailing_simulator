import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as lines
from math import cos, sin, pi
from utils import wrapTo2Pi

def draw_SailBoat(h, s, x, y, a_b, a_r, a_s):
	distance_rudder   = -11
	distance_sail     = 3
	# s - scale of boat, equals width
	# x - coordinate
	# y - coordinate
	# a_b - boats heading
	# a_r - rudder angle in b-frame
	# a_s - sail angle in b-frame

	hull              = np.array([[13, 3,-12,-12, 3,13],
								  [ 0,-2, -1,  1, 2, 0],
								  [ 1, 1,  1,  1, 1, 1]])
		
	rotation_hull     = np.array([[ np.cos(a_b),-np.sin(a_b), x],
								  [ np.sin(a_b), np.cos(a_b), y],
									  [           0,           0, 1]])
		
		
	rudder            = np.array([[0,-3],
								  [0, 0],
								  [1, 1]])

	rotation_rudder   = np.array([[np.cos(a_b+a_r),-np.sin(a_b+a_r), x+distance_rudder*cos(a_b)],
								  [np.sin(a_b+a_r), np.cos(a_b+a_r), y+distance_rudder*sin(a_b)],
								  [              0,               0,                        1]])


	sail              = np.array([[0,-10],
								  [0,  0],
								  [1,  1]])

	rotation_sail     = np.array([[np.cos(a_b+a_s),-np.sin(a_b+a_s), x+distance_sail*cos(a_b)],
								  [np.sin(a_b+a_s), np.cos(a_b+a_s), y+distance_sail*sin(a_b)],
								  [              0,               0,                      1]])

	hull              = s*rotation_hull.dot(hull)
	rudder            = s*rotation_rudder.dot(rudder)
	sail              = s*rotation_sail.dot(sail)
		
	plt.plot(hull[0,:],hull[1,:],'k')
	plt.plot(rudder[0,:],rudder[1,:],'b')
	plt.plot(sail[0,:],sail[1,:],'g')
		

def draw_WingBoat(h,s,x,y,a_b,a_r,MWAngle=0,tailAngle=0):
	distance_rudder   = -11
	distance_MW       = 3
	distance_tail     = -6

		
	hull              = np.array([[13, 3,-12,-12, 3,13],
								  [ 0,-2, -1,  1, 2, 0],
								  [ 1, 1,  1,  1, 1, 1]])
		
	MW                = np.array([[ 2, 0,-3, 0, 2],
								  [ 0,-1, 0, 1, 0],
								  [ 1, 1, 1, 1, 1]])
		
	tailWing          = np.array([[ 1,   0,-1.5,  0, 1],
								  [ 0,-0.5,   0,0.5, 0],
								  [ 1,   1   ,1,  1, 1]])
		
	rudder            = np.array([[0,-3],
								  [0, 0],
								  [1, 1]])

	rotation_hull     = np.array([[ np.cos(a_b),-np.sin(a_b), x],
								  [ np.sin(a_b), np.cos(a_b), y],
								  [           0,           0, 1]])
		
	rotation_rudder   = np.array([[np.cos(a_b+a_r),-np.sin(a_b+a_r), x+distance_rudder*cos(a_b)],
								  [np.sin(a_b+a_r), np.cos(a_b+a_r), y+distance_rudder*sin(a_b)],
								  [              0,               0,                        1]])
		
	rotation_MW       = np.array([[np.cos(a_b+MWAngle), -np.sin(a_b+MWAngle), x+distance_MW*cos(a_b)],
								  [np.sin(a_b+MWAngle),  np.cos(a_b+MWAngle), y+distance_MW*sin(a_b)],
								  [                  0,                    0,                      1]])

	rotation_tailWing = np.array([[np.cos(a_b+MWAngle+tailAngle), -np.sin(a_b+MWAngle+tailAngle), x+distance_MW*cos(a_b)+distance_tail*cos(a_b+MWAngle)],
								  [np.sin(a_b+MWAngle+tailAngle),  np.cos(a_b+MWAngle+tailAngle), y+distance_MW*sin(a_b)+distance_tail*sin(a_b+MWAngle)],
								  [                            0,                              0,                                                     1]])

	hull              = s*rotation_hull.dot(hull)
	MW                = s*rotation_MW.dot(MW)
	tailWing          = s*rotation_tailWing.dot(tailWing)
	rudder            = s*rotation_rudder.dot(rudder)
		
	plt.plot(hull[0,:],hull[1,:],'k')
	plt.plot(MW[0,:],MW[1,:],'r')
	plt.plot(x+distance_MW*cos(a_b),y+distance_MW*sin(a_b),'r+')
	plt.plot(tailWing[0,:],tailWing[1,:],'g')
	plt.plot( x+distance_MW*cos(a_b)+distance_tail*cos(a_b+MWAngle), y+distance_MW*sin(a_b)+distance_tail*sin(a_b+MWAngle),'g+')
	plt.plot(rudder[0,:],rudder[1,:],'b')



def draw_wind_direction(h, axis_min, axis_max_l, s, psi):
	# h  - actual fig add_axes
	# axis_min - minimum of axis (x,y)
	# axis_max_l  - length of figure
	# s - scale of boat, equals width
	# psi - true wind direction
	m_x = axis_min[0]+axis_max_l/2
	m_y = axis_min[1]+axis_max_l/2


	arrow          = np.array([[ 0, 2, 1.5, 2, 1.5],
							   [ 0, 0, 0.5, 0,-0.5],
							   [ 1, 1,   1, 1,   1]])

	rotation_arrow = np.array([[ np.cos(psi), -np.sin(psi), m_x],
							   [ np.sin(psi),  np.cos(psi), m_y],
							   [           0,            0,   1]])

	arrow          = rotation_arrow.dot(arrow)

	plt.plot(arrow[0,:],arrow[1,:],'b')




def draw_line(h, a, b, color_line='b'):
	l1 = lines.Line2D([a[0], b[0]], [a[1], b[1]], color=color_line)
	h.add_line(l1)


if __name__ == '__main__':
	fig = plt.figure()
	fig.subplots_adjust(top=0.8)

	ax2 = fig.add_axes([0.1, 0.1, 0.8, 0.8])

	ax2.set_xlabel('Simulation of boat')

	draw_boat(ax2, 0.1, 3, 4, 1, 0.5, 0.5)
	draw_wind_direction(ax2, (0, 0), 5, 0.1, pi)
	draw_line(ax2, (1, 1), (2, 2), color_line='r')
	plt.axis([0, 5, 0, 5])
	plt.show()
