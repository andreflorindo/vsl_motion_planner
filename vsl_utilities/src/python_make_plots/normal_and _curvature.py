# coding=utf-8

import sys
import numpy as np
import math
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline, make_interp_spline, splprep, splev
from scipy.signal import argrelextrema


class CourseClass:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def read_path():
    input = np.loadtxt(
        #"/home/andreflorindo/workspaces/tesseract_vsl_motion_planner_ws/src/vsl_motion_planner/vsl_msgs/examples/simplePath.txt", dtype='f')
        "/home/andre/workspaces/tesseract_ws/src/vsl_motion_planner/vsl_msgs/examples/weirdPath.txt", dtype='f')
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


def plot_three_courses2d(course, bspline, bsplinetck):
    pyplot.figure()
    pyplot.title('Cartesian Path')
    pyplot.ylabel('y(m)')
    pyplot.xlabel('x(m)')
    pyplot.plot(course.x, course.y, label='Course', marker='.',
            color='red', linestyle='dashed', markerfacecolor='yellow')
    pyplot.plot(bspline.x, bspline.y, label='5rd order Spline',
            color='blue', markerfacecolor='yellow')
    pyplot.plot(bsplinetck.x, bsplinetck.y, label='3rd order Spline',
            color='green', markerfacecolor='yellow')
    pyplot.legend()
    pyplot.show()


def plot_dev_courses2d(course, bspline, bsplinetck):
    pyplot.figure()
    pyplot.title('Cartesian Path')
    pyplot.ylabel('y(m)')
    pyplot.xlabel('x(m)')
    pyplot.plot(course.x, course.y, label='Course', marker='.',
            color='red', linestyle='dashed', markerfacecolor='yellow')
    pyplot.plot(bspline.x, bspline.y, label='5rd order Spline',
            color='blue', markerfacecolor='yellow')
    pyplot.plot(bspline.x, bsplinetck.y, label='Dev1 5rd order Spline',
            color='green', markerfacecolor='yellow')
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
    #Using Bspline
    xd = BSpline(u, course.x, k)
    yd = BSpline(u, course.y, k)
    zd = BSpline(u, course.z, k)
    bspline_x = xd(parameter)
    bspline_y = yd(parameter)
    bspline_z = zd(parameter)
    bspline_course = CourseClass(bspline_x, bspline_y, bspline_z)
    return bspline_course


def bspline3Dtck(course, degree):

    #Using make_interp_spline
    #bspline_x = np.linspace(course.x[0], course.x[len(course.x)-1], num=88)
    #b = make_interp_spline(course.x, course.y)
    #bspline_y = b(bspline_x)
    #print(np.allclose(bspline_y, course.y))
    
    #Another Using make_interp_spline and splprep
    #tck, u = splprep([course.x,course.y,course.z], s=0.0, k=5, nest=-1)
    #l, r = [(1, (0, 0, 0))], [(2, (0, 0, 0))]
    #clamped_spline = make_interp_spline(u, np.array([course.x,course.y,course.z]).T, bc_type=(l, r))
    #bspline_x, bspline_y, bspline_z = clamped_spline(np.linspace(0,1,100)).T

    #Using splprep

    tck, u = splprep([course.x,course.y,course.z], k=degree, s=0.000001) #0.000001
    arc_length = compute_arc_length(course)
    n_waypoints = int(arc_length // 0.0001)
    u_new = np.linspace(u.min(), u.max(), n_waypoints)
    bspline_x, bspline_y, bspline_z = splev(u_new, tck, der=0)

    bspline_course = CourseClass(bspline_x, bspline_y, bspline_z)

    return bspline_course

def bspline3Dtck_iterative(course, degree):
    smooth=0.0000001
    proceed=False
    while(proceed==False):
        tck, u = splprep([course.x,course.y,course.z], k=degree, s=smooth) #0.000001
        arc_length = compute_arc_length(course)
        n_waypoints = int(arc_length // 0.0001)
        u_new = np.linspace(u.min(), u.max(), n_waypoints)
        bspline_x, bspline_y, bspline_z = splev(u_new, tck, der=0)
        deriv_bspline_x, deriv_bspline_y, deriv_bspline_z = splev(u_new, tck, der=2)
        if(check_smoothness(deriv_bspline_y) == True):
            proceed = True
        else:
            smooth=smooth+0.0000001
            proceed= False
    print(smooth)
    bspline_course = CourseClass(bspline_x, bspline_y, bspline_z)

    return bspline_course, smooth

def check_smoothness(course_x):
    proceed = True
    course_x_array = np.asarray(course_x)
    maxInd = argrelextrema(course_x_array, np.greater)
    if len(maxInd[0]) > 5:
        proceed = False
    return proceed


def deriv_bspline3D(order, parameter, u, course, k,degree,smooth):
    #xd = BSpline(u, course.x, k)
    #yd = BSpline(u, course.y, k)
    #zd = BSpline(u, course.z, k)

    #deriv_xd = xd.derivative(order)
    #deriv_yd = yd.derivative(order)
    #deriv_zd = zd.derivative(order)

    #deriv_bspline_x = deriv_xd(parameter)
    #deriv_bspline_y = deriv_yd(parameter)
    #deriv_bspline_z = deriv_zd(parameter)


    tck, u = splprep([course.x,course.y,course.z], k=degree, s=smooth) #0.000001
    arc_length = compute_arc_length(course)
    n_waypoints = int(arc_length // 0.0001)
    u_new = np.linspace(u.min(), u.max(), n_waypoints)
    deriv_bspline_x, deriv_bspline_y, deriv_bspline_z = splev(u_new, tck, der=order)

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


def interpolate_course(course):
    tck, u = splprep([course.x,course.y,course.z], k=3, s=0.000000)
    n_waypoints = 1630
    u_new = np.linspace(u.min(), u.max(), n_waypoints)
    inter_x, inter_y, inter_z = splev(u_new, tck, der=0)
    inter_course = CourseClass(inter_x, inter_y, inter_z)

    return inter_course


def compute_position_error(course, bspline):
    inter_course = interpolate_course(course)
    inter_robot_pose = interpolate_course(bspline)

    absolute_error = []

    if(len(inter_course.x) != len(inter_robot_pose.x)):
        print("Paths have not the same number of point, error may be bad",len(inter_course.x),len(inter_robot_pose.x))
    
    for i in range (0,len(inter_robot_pose.x)):
        error_x = abs(inter_robot_pose.x[i]-inter_course.x[i])
        error_y = abs(inter_robot_pose.y[i]-inter_course.y[i])
        error_z = abs(inter_robot_pose.z[i]-inter_course.z[i])
        absolute_error.append(math.sqrt(error_x**2+error_y**2+error_z**2)*1000)
    
    return inter_robot_pose.x, absolute_error

def plot_error_one_file(x, absolute_error_3degree, absolute_error_5degree, absolute_error_bspline):   
    pyplot.figure()
    pyplot.title('Absolute Position error')
    pyplot.ylabel('Absolute error (mm)')
    pyplot.xlabel('x(m)')
    pyplot.plot(x, absolute_error_3degree, 'g',label='3rd Degree Bspline')
    pyplot.plot(x, absolute_error_5degree, 'r',label='5th Degree Bspline')
    pyplot.plot(x, absolute_error_bspline, 'b',label='Nth Degree Bspline')
    pyplot.legend()
    pyplot.show()

if __name__ == "__main__":

    # m+1 knots vector elements
    # n+1 control points
    # k curve degree

    course = read_path()
    arc_length=compute_arc_length(course)
    # plot_course(course)

    #k = 4
    #u_middle=0.0
    #number_u_points = len(course.x)-(k-1)
    #u = []
    #for i in range(0, k+len(course.x)):
    #    if i < k:
    #        u.append(0)
    #    elif i > (k+len(course.x))-(k+1):
    #        u.append(1)
    #    else:
    #        u_middle=u_middle+float(1.0/number_u_points)
    #        u.append(u_middle)

    k = len(course.x)-1
    u = []
    for i in range(0, 2*k+2):
        if i < k+1:
            u.append(0)
        else:
            u.append(1)
    n_waypoints = int(arc_length // 0.0001)

    parameter = np.linspace(0, 1, num=n_waypoints)

    bspline_course = bspline3D(parameter, u, course, k)
    position = recognize_position(course, bspline_course)

    #deriv1_bspline_position = deriv_bspline_position(
    #    1, position, parameter, u, course, k)
    #deriv2_bspline_position = deriv_bspline_position(
    #    2, position, parameter, u, course, k)
    #tangent = build_vector(deriv1_bspline_position)
    #normal = build_vector(deriv2_bspline_position)

    #bspline_course_tck_3 = bspline3Dtck(course,3)
    bspline_course_tck_3, smooth3 = bspline3Dtck_iterative(course,3)
    deriv_bspline_course_tck_3 = deriv_bspline3D(1,parameter, u, course, k, 3, smooth3)

    #bspline_course_tck_5 = bspline3Dtck(course,5)
    bspline_course_tck_5, smooth5 = bspline3Dtck_iterative(course,5)
    deriv_bspline_course_tck_5 = deriv_bspline3D(1,parameter, u, course, k, 5, smooth5)


    plot_three_courses2d(course,  bspline_course_tck_5, bspline_course_tck_3)
    plot_dev_courses2d(course, bspline_course_tck_5, deriv_bspline_course_tck_5)

    #binormal = []
    #for i in range(0, len(tangent)):
    #    binormal_vector = np.cross(tangent[i], normal[i])
    #    binormal_norm = math.sqrt(
    #        binormal_vector[0]**2+binormal_vector[1]**2+binormal_vector[2]**2)
    #    if binormal_vector[2] < 0:
    #        binormal_vector[0] = -binormal_vector[0]
    #        binormal_vector[1] = -binormal_vector[1]
    #        binormal_vector[2] = -binormal_vector[2]
    #    binormal.append(binormal_vector/binormal_norm)

    # binormal=[]
    # for i in range(0,len(tangent)):
    #     binormal.append([0,0,1])

    #np.savetxt("/home/andre/workspaces/tesseract_ws/src/vsl_motion_planner/vsl_msgs/examples/tangent_simplePath.txt", tangent, fmt='%.6f')
    #np.savetxt("/home/andre/workspaces/tesseract_ws/src/vsl_motion_planner/vsl_msgs/examples/normal_simplePath.txt", normal, fmt='%.6f')
    #np.savetxt("/home/andre/workspaces/tesseract_ws/src/vsl_motion_planner/vsl_msgs/examples/binormal_simplePath.txt", binormal, fmt='%.6f')

    # plot_course(bspline_course)
    #plot_both_courses2d(course, bspline_course)
    #plot_three_courses2d(course, bspline_course_tck_5, bspline_course_tck_3)
    
    # radius = compute_radius2D(deriv1_bspline_position,deriv2_bspline_position)
    # print(radius)

    # deriv_bspline_course = deriv_bspline3D(1,parameter, u, course, k)
    # deriv2_bspline_course = deriv_bspline3D(2,parameter, u, course, k)
    # plot_both_courses2d(deriv_bspline_course, deriv2_bspline_course)

    x, error_3 = compute_position_error(course, bspline_course_tck_3)
    x, error_5 = compute_position_error(course, bspline_course_tck_5)
    x, error_bspline = compute_position_error(course, bspline_course)
    plot_error_one_file(x, error_3, error_5, error_bspline)

