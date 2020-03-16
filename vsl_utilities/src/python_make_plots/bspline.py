# coding=utf-8

import sys
import numpy as np
import math
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D


class CourseClass:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def read_path():
    input = np.loadtxt(
        #"/home/andreflorindo/workspaces/vsl_msc_project_ws/src/vsl_msgs/examples/simplePath.txt", dtype='f')
        "/home/andre/workspaces/tesseract_ws/src/vsl_motion_planner/vsl_msgs/examples/simplePath.txt", dtype='f')
    x = []
    y = []
    z = []
    for i in range(0, len(input)):
        x.append(input[i][0])
        y.append(input[i][1])
        z.append(input[i][2])

    course = CourseClass(x, y, z)
    return course


def plot_course(course):
    # 3D plotting setup
    fig = pyplot.figure()
    ax = fig.add_subplot(111, projection='3d')
    axis_size = 0.5
    ax.plot(course.x, course.y, course.z, label='Course', marker='.',
            color='blue', linestyle='dashed', markerfacecolor='yellow')
    ax.legend()
    ax.set_xlabel('X')
    ax.set_xlim(0, axis_size)
    ax.set_ylabel('Y')
    ax.set_ylim(0, axis_size)
    ax.set_zlabel('Z')
    ax.set_zlim(-axis_size/2, axis_size/2)
    pyplot.show()


def B(x, k, i, u):
    if k == 0:
        return 1.0 if u[i] <= x < u[i+1] else 0.0

    if u[i+k] == u[i]:
        c1 = 0.0
    else:
        c1 = (x - u[i])/(u[i+k] - u[i]) * B(x, k-1, i, u)

    if u[i+k+1] == u[i+1]:
        c2 = 0.0
    else:
        c2 = (u[i+k+1] - x)/(u[i+k+1] - u[i+1]) * B(x, k-1, i+1, u)
    return c1 + c2


def bspline1D(x, u, p, k):
    m = len(u)
    n = m - k - 1
    bspline = 0
    for i in range(0,n):
        add_element = p[i] * B(x, k, i, u)
        bspline = add_element + bspline
        print(bspline)

    return bspline


def bspline3D(x, u, course, k):
    bx = []
    by = []
    bz = []
 
    for j in range(0, len(x)):
        xd = bspline1D(x[j], u, course.x, k)
        yd = bspline1D(x[j], u, course.y, k)
        zd = bspline1D(x[j], u, course.z, k)
        bx.append(xd)
        by.append(yd)
        bz.append(zd)

    bspline_course = CourseClass(bx, by, bz)
    return bspline_course


if __name__ == "__main__":

    # m+1 knots vector elements
    # n+1 control points
    # k curve degree

    course = read_path()
    # plot_course(course)

    k = len(course.x)-1

    u=[]
    for i in range(0,2*k+2):
         if i<k:
             u.append(0)
         else:
             u.append(1)

    parameter = np.linspace(0, 1, num=100)

    bspline_course = bspline3D(parameter, u, course, k)
    plot_course(bspline_course)
