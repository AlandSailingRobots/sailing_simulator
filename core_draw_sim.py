import numpy as np
import matplotlib.pylab as plt
import matplotlib.lines as lines
import matplotlib.cm as cm
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
    points = [p1,p2,[x, y],p3]
    poly = plt.Polygon(points, fill=None, edgecolor='k', linewidth=0.5)
    h.add_patch(poly)
    # l1 = lines.Line2D([p2[0], p1[0]], [p2[1], p1[1]], color='k')
    # l2 = lines.Line2D([p3[0], p1[0]], [p3[1], p1[1]], color='k')
    # l3 = lines.Line2D([p2[0], p3[0]], [p2[1], p3[1]], color='k')
    # l4 = lines.Line2D([pr1[0], pr2[0]], [pr1[1], pr2[1]], color='k')   # rudder
    # l5 = lines.Line2D([x, ps[0]], [y, ps[1]], color='k')       # sail
    # h.add_line(l1)
    # h.add_line(l2)
    # h.add_line(l3)
    # h.add_line(l4)
    # h.add_line(l5)
    # l1.remove()
    # l2.remove()
    # l3.remove()
    # l4.remove()
    # l5.remove()


def draw_track(h, a, b, color_='r', width_=0.5):
    # h  - actual fig add_axes
    # a  - longitude
    # b  - latitude
    l1 = lines.Line2D([a[0], b[0]], [a[1], b[1]], color=color_, linewidth=width_)
    # line.add_line(lines.Line2D([a[0], b[0]], [a[1], b[1]], color=color_, linewidth=width_))
    h.add_line(l1)


def draw_ais_track(h, a, b, dist):
    # h  - actual fig add_axes
    # a  - longitude
    # b  - latitude
    def_dist = 100
    cgrad = def_dist/dist
    print(dist)
    # print(cgrad)
    l1 = lines.Line2D([a[1], b[1]], [a[0], b[0]],color=cm.jet(cgrad),linewidth=0.5)
    h.add_line(l1)
    return a


def draw_ais(h, s, pos, d, color_='b', width_=0.5):
    # h  -  figure axes
    # s  -  scale
    # x  -  longitude
    # y  -  latitude
    # d  -  direction
    (y, x) = pos
    # print(x, y)
    p1 = [x+s*cos(d), y+s*sin(d)]
    p1 = [x, y]
    p2 = [x-s*cos(d)*2+s/2*cos(pi/2-d), y-s*sin(d)*2-s/2*sin(pi/2-d)]
    p3 = [x-s*cos(d)*2-s/2*cos(pi/2-d), y-s*sin(d)*2+s/2*sin(pi/2-d)]
    p = [p1, p2, p3]
    poly = plt.Polygon(p, fill=None, edgecolor='b', linewidth=width_)
    h.add_patch(poly)
    # l1 = lines.Line2D([p2[0], p1[0]], [p2[1], p1[1]], color=color_, linewidth=width_)
    # l2 = lines.Line2D([p3[0], p1[0]], [p3[1], p1[1]], color=color_, linewidth=width_)
    # l3 = lines.Line2D([p2[0], p3[0]], [p2[1], p3[1]], color=color_, linewidth=width_)
    # h.add_line(l1)
    # h.add_line(l2)
    # h.add_line(l3)


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


if __name__ == '__main__':
    fig = plt.figure()
    fig.subplots_adjust(top=0.8)

    ax2 = fig.add_axes([0.1, 0.1, 0.8, 0.8])

    ax2.set_xlabel('Simulation of boat')

    draw_boat(ax2, 0.1, 3, 4, 1, 0.5, 0.5)
    draw_wind_direction(ax2, (0, 0), 5, 0.1, pi)
    draw_line(ax2, (1, 1), (2, 2), color_line='r')
    draw_ais(ax2, 0.15, (1,1), 0)
    plt.axis([0, 5, 0, 5])
    plt.show()
