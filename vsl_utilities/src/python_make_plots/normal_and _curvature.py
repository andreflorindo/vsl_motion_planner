# coding=utf-8

import sys
import numpy as np
import math
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline


class CourseClass:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def read_path():
    input = np.loadtxt(
        "/home/andreflorindo/workspaces/tesseract_vsl_motion_planner_ws/src/vsl_motion_planner/vsl_msgs/examples/simplePath.txt", dtype='f')
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




def plot_both_courses2d(course, bspline):
    pyplot.figure()
    pyplot.title('Cartesian Path')
    pyplot.ylabel('y(m)')
    pyplot.xlabel('x(m)')
    pyplot.plot(course.x, course.y, label='Course', marker='.',
            color='red', linestyle='dashed', markerfacecolor='yellow')
    pyplot.plot(bspline.x, bspline.y, label='Bspline',
            color='blue', linestyle='dashed', markerfacecolor='yellow')
    pyplot.legend()
    pyplot.show()


def plot_both_courses3d(course, bspline):
    # 3D plotting setup
    fig = pyplot.figure()
    ax = fig.add_subplot(111, projection='3d')
    axis_size = 0.5
    ax.plot(course.x, course.y, course.z, label='Course', marker='.',
            color='red', linestyle='dashed', markerfacecolor='yellow')
    ax.plot(bspline.x, bspline.y, bspline.z, label='Bspline',
            color='blue', linestyle='dashed', markerfacecolor='yellow')
    ax.legend()
    ax.set_xlabel('X')
    ax.set_xlim(0, axis_size)
    ax.set_ylabel('Y')
    ax.set_ylim(0, axis_size)
    ax.set_zlabel('Z')
    ax.set_zlim(-axis_size/2, axis_size/2)
    pyplot.show()


def bspline3D(parameter, u, course, k):

    xd = BSpline(u, course.x, k)
    yd = BSpline(u, course.y, k)
    zd = BSpline(u, course.z, k)

    bspline_x = xd(parameter)
    bspline_y = yd(parameter)
    bspline_z = zd(parameter)

    bspline_course = CourseClass(bspline_x, bspline_y, bspline_z)

    return bspline_course


def deriv_bspline3D(order, parameter, u, course, k):
    xd = BSpline(u, course.x, k)
    yd = BSpline(u, course.y, k)
    zd = BSpline(u, course.z, k)

    deriv_xd = xd.derivative(order)
    deriv_yd = yd.derivative(order)
    deriv_zd = zd.derivative(order)

    deriv_bspline_x = deriv_xd(parameter)
    deriv_bspline_y = deriv_yd(parameter)
    deriv_bspline_z = deriv_zd(parameter)

    deriv_bspline_course = CourseClass(
        deriv_bspline_x, deriv_bspline_y, deriv_bspline_z)
    return deriv_bspline_course


def recognize_position(course, bspline_course):
    position = []
    for j in range(0, len(course.x)):
        percentage = 0.2
        for i in range(0, len(bspline_course.x)):
            if bspline_course.x[i] >= (1-percentage)*course.x[j] and bspline_course.x[i] <= (1+percentage)*course.x[j]:
                if bspline_course.x[i] > course.x[j]:
                    percentage = bspline_course.x[i]/course.x[j]-1
                else:
                    percentage = 1-bspline_course.x[i]/course.x[j]
                k = i
        position.append(k)
    return position


def deriv_bspline_position(order, position, parameter, u, course, k):
    xd = BSpline(u, course.x, k)
    yd = BSpline(u, course.y, k)
    zd = BSpline(u, course.z, k)

    deriv_xd = xd.derivative(order)
    deriv_yd = yd.derivative(order)
    deriv_zd = zd.derivative(order)

    deriv_parameter = []
    for i in range(0, len(position)):
        deriv_parameter.append(parameter[position[i]])

    deriv_bspline_x = deriv_xd(deriv_parameter)
    deriv_bspline_y = deriv_yd(deriv_parameter)
    deriv_bspline_z = deriv_zd(deriv_parameter)

    deriv_bspline_course = CourseClass(
        deriv_bspline_x, deriv_bspline_y, deriv_bspline_z)
    return deriv_bspline_course


def build_vector(deriv_bspline):
    deriv_vector = []
    for i in range(0, len(deriv_bspline.x)):
        vector_norm = math.sqrt(
            deriv_bspline.x[i]**2+deriv_bspline.y[i]**2+deriv_bspline.z[i]**2)
        vector = [(deriv_bspline.x[i])/vector_norm, (deriv_bspline.y[i]
                                                     )/vector_norm, (deriv_bspline.z[i])/vector_norm]
        deriv_vector.append(vector)
    return deriv_vector


def compute_arc_length(course):
    arc_length = 0
    for i in range(1, len(course.x)):
        arc_length = arc_length + math.sqrt((course.x[i]-course.x[i-1])**2 + (
            course.y[i]-course.y[i-1])**2+(course.z[i]-course.z[i-1])**2)
    print('Arc Length: ', arc_length)
    return arc_length


def compute_radius2D(dp, ddp):
    radius = []
    for i in range(0, len(dp.x)):
        num = math.sqrt(dp.x[i]**2+dp.y[i]**2)**3
        denom = dp.x[i]*ddp.y[i]-dp.y[i]*ddp.x[i]
        radius.append(abs(num/denom))
    return radius

def compute_radius3D(dp, ddp):
    radius = []
    for i in range(0, len(dp.x)):
        num = math.sqrt(dp.x[i]**2+dp.y[i]**2+dp.z[i]**2)**3
        denom = math.sqrt((dp.y[i]*ddp.z[i]-ddp.y[i]*dp.z[i])**2+(dp.x[i]*ddp.z[i]-ddp.x[i]*dp.z[i])**2+(dp.x[i]*ddp.y[i]-dp.y[i]*ddp.x[i])**2)
        radius.append(abs(num/denom))
    return radius



if __name__ == "__main__":

    # m+1 knots vector elements
    # n+1 control points
    # k curve degree

    course = read_path()
    compute_arc_length(course)
    # plot_course(course)

    k = len(course.x)-1

    u = []
    for i in range(0, 2*k+2):
        if i < k+1:
            u.append(0)
        else:
            u.append(1)

    parameter = np.linspace(0, 1, num=100000)

    bspline_course = bspline3D(parameter, u, course, k)
    position = recognize_position(course, bspline_course)

    deriv1_bspline_position = deriv_bspline_position(
        1, position, parameter, u, course, k)
    deriv2_bspline_position = deriv_bspline_position(
        2, position, parameter, u, course, k)
    tangent = build_vector(deriv1_bspline_position)
    normal = build_vector(deriv2_bspline_position)

    binormal = []
    for i in range(0, len(tangent)):
        binormal_vector = np.cross(tangent[i], normal[i])
        binormal_norm = math.sqrt(
            binormal_vector[0]**2+binormal_vector[1]**2+binormal_vector[2]**2)
        if binormal_vector[2] < 0:
            binormal_vector[0] = -binormal_vector[0]
            binormal_vector[1] = -binormal_vector[1]
            binormal_vector[2] = -binormal_vector[2]
        binormal.append(binormal_vector/binormal_norm)

    # binormal=[]
    # for i in range(0,len(tangent)):
    #     binormal.append([0,0,1])

    # np.savetxt("/home/andreflorindo/workspaces/vsl_motion_planner_ws/src/vsl_utilities/examples/tangent_simplePath.txt", tangent, fmt='%.6f')
    # np.savetxt("/home/andreflorindo/workspaces/vsl_motion_planner_ws/src/vsl_utilities/examples/normal_simplePath.txt", normal, fmt='%.6f')
    # np.savetxt("/home/andreflorindo/workspaces/vsl_motion_planner_ws/src/vsl_utilities/examples/binormal_simplePath.txt", binormal, fmt='%.6f')

    # plot_course(bspline_course)
    # plot_both_courses2d(course, bspline_course)
    # radius = compute_radius2D(deriv1_bspline_position,deriv2_bspline_position)
    # print(radius)

    # deriv_bspline_course = deriv_bspline3D(1,parameter, u, course, k)
    # deriv2_bspline_course = deriv_bspline3D(2,parameter, u, course, k)
    # plot_both_courses2d(deriv_bspline_course, deriv2_bspline_course)
