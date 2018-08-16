import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.cm as cm
from math import cos, sin, pi
from utils import wrapTo2Pi, degTorad


def draw_boat(h, s, x, y, a_b, a_r, a_s):
    # s - scale of boat, equals width
    # x - coordinate
    # y - coordinate
    # a_b - boats heading
    # a_r - rudder angle in b-frame
    # a_s - sail angle in b-frame
    # Calculate corners

    p1 = [x+s*cos(a_b), y+s*sin(a_b)]
    p2 = [x-s*cos(a_b)+s/2*cos(pi/2-a_b), y-s*sin(a_b)-s/2*sin(pi/2-a_b)]
    p3 = [x-s*cos(a_b)-s/2*cos(pi/2-a_b), y-s*sin(a_b)+s/2*sin(pi/2-a_b)]
    #pr1 = [x-s*cos(a_b), y-s*sin(a_b)]
    #pr2 = [x-s*cos(a_b)-s*cos(a_b+a_r), y-s*sin(a_b)-s*sin(a_b+a_r)]
    #ps = [x-s*cos(a_b+a_s), y-s*sin(a_b+a_s)]
    points = [p1,p2,[x, y],p3]
    poly = plt.Polygon(points, fill=None, edgecolor='k', linewidth=0.5)
    h.add_patch(poly)


def draw_track(h, a, b, d, width_=1):
  # these comments are wrong !!!!!!
    # h  - actual fig add_axes
    # a  - longitude
    # b  - latitude
    # d  - distance
    def_dist = 100
    cgrad = def_dist/d
    l1 = lines.Line2D([a[1], b[1]], [a[0], b[0]], color=cm.magma(cgrad), linewidth=width_)
    h.add_line(l1)
    return a


def draw_ais_track(h, a, b, d):
  # these comments are wrong !!!!!!
    # h  - actual fig add_axes
    # a  - longitude
    # b  - latitude
    # d  - distance
    def_dist = 100
    cgrad = def_dist/d
    l1 = lines.Line2D([a[1], b[1]], [a[0], b[0]],color=cm.jet(cgrad),linewidth=0.5)
    h.add_line(l1)
    return a


def draw_ais(h, s, pos, d, color_='b', width_=0.5):
    # h  -  figure axes
    # s  -  scale
    # x  -  longitude
    # y  -  latitude
    # d  -  direction
    #d = 0
    #d = -np.pi/2
    
    (y, x) = pos
    p1 = [x+s*cos(d), y+s*sin(d)]
    p2 = [x-s*cos(d)+(s/2)*cos(pi/2-d), y-s*sin(d)-(s/2)*sin(pi/2-d)]
    p3 = [x-s*cos(d)-(s/2)*cos(pi/2-d), y-s*sin(d)+(s/2)*sin(pi/2-d)]
    p = [p1, p2, p3]
    
    poly = plt.Polygon(p, fill=None, edgecolor='b', linewidth=width_)
    h.add_patch(poly)


def draw_wind_direction(h, axis_min, axis_max_l, s, psi):
    # h  - actual fig add_axes
    # axis_min - minimum of axis (x,y)
    # axis_max_l  - length of figure
    # s - scale of boat, equals width
    # psi - true wind direction
    m_x = axis_min[0]+axis_max_l/2
    m_y = axis_min[1]+axis_max_l/1

    x_w = [m_x,
           m_x+1*s*cos(psi),
           m_x+1*s*cos(psi)-s*cos(psi-pi/4),
           m_x+1*s*cos(psi)-s*cos(psi+pi/4)]
    y_w = [m_y,
           m_y+1*s*sin(psi),
           m_y+1*s*sin(psi)-s*sin(psi-pi/4),
           m_y+1*s*sin(psi)-s*sin(psi+pi/4)]
    l1 = lines.Line2D([x_w[0], x_w[1]], [y_w[0], y_w[1]], color='w')
    l2 = lines.Line2D([x_w[1], x_w[2]], [y_w[1], y_w[2]], color='w')
    l3 = lines.Line2D([x_w[1], x_w[3]], [y_w[1], y_w[3]], color='w')
    h.add_line(l1)
    h.add_line(l2)
    h.add_line(l3)


def draw_wp(h, s, x, y, r):
    # h  -  figure axes
    # s  -  scale
    # x  -  longitude (center)
    # y  -  latitude (center)
    # r  -  radius
    radius_degree = (r+3)*1.132*1e-5
    circle1 = plt.Circle((x,y), radius_degree/10, color='w')
    circle2 = plt.Circle((x,y), radius_degree, fill=False)
    h.add_artist(circle1)
    h.add_artist(circle2)


def draw_line(h, a, b, color_line='b'):
    l1 = lines.Line2D([a[0], b[0]], [a[1], b[1]], color=color_line, linestyle='--', dashes=(2,4))
    # l1.set_dashes('-')
    h.add_line(l1)


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
    plt.sca(h)

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

    rotation_tailWing = np.array([[np.cos(a_b+MWAngle+tailAngle), -np.sin(a_b+MWAngle+tailAngle), x+distance_MW*cos(a_b)+distance_tail*cos(MWAngle+a_b)],
                                  [np.sin(a_b+MWAngle+tailAngle),  np.cos(a_b+MWAngle+tailAngle), y+distance_MW*sin(a_b)+distance_tail*sin(MWAngle+a_b)],
                                  [                            0,                              0,                              1]])

    hull              = s*rotation_hull.dot(hull)
    MW                = s*rotation_MW.dot(MW)
    tailWing          = s*rotation_tailWing.dot(tailWing)
    rudder            = s*rotation_rudder.dot(rudder)

    plt.plot(hull[0,:],hull[1,:],'k')
    plt.plot(MW[0,:],MW[1,:],'r')
    plt.plot(x+distance_MW*cos(a_b),y+distance_MW*sin(a_b),'r+')
    plt.plot(tailWing[0,:],tailWing[1,:],'g')
    # plt.plot( x+distance_MW*cos(a_b)+distance_tail*cos(MWAngle), y+distance_MW*sin(a_b)+distance_tail*sin(MWAngle),'g+')
    # plt.plot([x+distance_MW*cos(a_b) ,x+distance_MW*cos(a_b)+distance_tail*cos(MWAngle)],[y+distance_MW*sin(a_b),y+distance_MW*sin(a_b)+distance_tail*sin(MWAngle)],'k')
    plt.plot(rudder[0,:],rudder[1,:],'b')


if __name__ == '__main__':
    fig = plt.figure()
    fig.subplots_adjust(top=0.8)

    ax2 = fig.add_axes([0.1, 0.1, 0.8, 0.8])

    ax2.set_xlabel('Simulation of boat')

    #draw_boat(ax2, 0.1, 3, 4, 1, 0.5, 0.5)
    draw_WingBoat(ax2, 1, 0, 0, 60, 0, 0, 0)
    #draw_wind_direction(ax2, (0, 0), 5, 0.1, pi)
    #draw_line(ax2, (1, 1), (2, 2), color_line='r')
    #draw_ais(ax2, 0.15, (1,1), 0)
    plt.axis([-20 ,20, -10, 10])
    plt.show()

