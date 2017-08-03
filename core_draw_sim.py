import numpy as np
import matplotlib.pylab as plt
import matplotlib.lines as lines
from math import cos, sin, pi


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
    pr1 = [x-s*cos(a_b), y-s*sin(a_b)]
    pr2 = [x-s*cos(a_b)-s*cos(a_b+a_r), y-s*sin(a_b)-s*sin(a_b+a_r)]
    ps = [x-s*cos(a_b+a_s), y-s*sin(a_b+a_s)]
    # draw lines
    l1 = lines.Line2D([p2[0], p1[0]], [p2[1], p1[1]], color='k')
    l2 = lines.Line2D([p3[0], p1[0]], [p3[1], p1[1]], color='k')
    l3 = lines.Line2D([p2[0], p3[0]], [p2[1], p3[1]], color='k')
    l4 = lines.Line2D([pr1[0], pr2[0]], [pr1[1], pr2[1]], color='k')   # rudder
    l5 = lines.Line2D([x, ps[0]], [y, ps[1]], color='k')       # sail
    h.add_line(l1)
    h.add_line(l2)
    h.add_line(l3)
    h.add_line(l4)
    h.add_line(l5)


def draw_track(h, t, x, y):
    # h  - actual fig add_axes
    # t  - vector with previous positions
    # x  - new x position
    # y  - new y position
    # t.append([x, y])
    # (xprev, yprev) = t[-1]
    # # for i in range(1, len(t)):
    # line = lines.Line2D([x, xprev], [y, yprev], color='r')
    # h.add_line(line)
    t.append([x, y])
    for i in range(1, len(t)):
        line = lines.Line2D([t[i][0], t[i-1][0]], [t[i][1], t[i-1][1]], color='r')
        h.add_line(line)


def draw_wind_direction(h, axis_min, axis_max_l, s, psi):
    # h  - actual fig add_axes
    # axis_min - minimum of axis (x,y)
    # axis_max_l  - length of figure
    # s - scale of boat, equals width
    # psi - true wind direction
    m_x = axis_min[0]+axis_max_l/2
    m_y = axis_min[1]+axis_max_l/2
    x_w = [m_x,
           m_x+3*s*cos(psi),
           m_x+3*s*cos(psi)-s*cos(psi-pi/4),
           m_x+3*s*cos(psi)-s*cos(psi+pi/4)]
    y_w = [m_y,
           m_y+3*s*sin(psi),
           m_y+3*s*sin(psi)-s*sin(psi-pi/4),
           m_y+3*s*sin(psi)-s*sin(psi+pi/4)]
    l1 = lines.Line2D([x_w[0], x_w[1]], [y_w[0], y_w[1]], color='b')
    l2 = lines.Line2D([x_w[1], x_w[2]], [y_w[1], y_w[2]], color='b')
    l3 = lines.Line2D([x_w[1], x_w[3]], [y_w[1], y_w[3]], color='b')
    h.add_line(l1)
    h.add_line(l2)
    h.add_line(l3)

def draw_circle(h, s, x, y, radius):
    # circle1 = plt.Circle((0, 0), 2, color='r')
    # now make a circle with no fill, which is good for hi-lighting key results
    # circle2 = plt.Circle((5, 5), 0.5, color='b', fill=False)
    radius_degree = (radius+3)*1.132*1e-5
    circle1 = plt.Circle((x,y), radius_degree/10, color='g')
    circle2 = plt.Circle((x,y), radius_degree, fill=False)
    h.add_artist(circle1)
    h.add_artist(circle2)


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
