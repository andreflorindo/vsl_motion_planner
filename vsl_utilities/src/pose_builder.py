#! /usr/bin/env python
# Author: Andre Florindo
# Requires Python 2.7

# Python
from __future__ import division
import sys
import numpy as np
import math
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline
# from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_from_matrix, rotation_matrix

# ROS
import rospy
import rospkg

# VSL
from vsl_msgs.srv import PoseBuilder, PoseBuilderResponse
from geometry_msgs.msg import Pose, PoseArray


class CourseClass:
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []


class PoseBuilderConfiguration:
    def __init__(self):
        self.world_frame = ''
        self.course_filename = ''
        self.distance_waypoints = []
        self.add_x_distance = []
        self.add_y_distance = []
        self.add_z_distance = []
        self.tow_width = []
        self.tow_thickness = []


class PoseBuilderPython:
    TABLE_HEIGHT = 0.78
    TABLE_WIDTH = 1.2
    APPROACH_TABLE = 0.002
    XY_EXTENSION_DISTANCE = 0.02  # meters
    XY_RAISE_DISTANCE = 0.05  # meters
    ANGLE_RAISE = 10  # degrees
    Z_RAISE_DISTANCE = XY_RAISE_DISTANCE*math.tan(ANGLE_RAISE*math.pi/180)
    SERVER_TIMEOUT = 5.0  # seconds
    POSE_BUILDER_SERVICE = "single_course"

    def __init__(self):
        self.config = PoseBuilderConfiguration()
        self.course = CourseClass()
        self.course_poses = PoseArray()

    def initServer(self):
        if rospy.has_param('~course_filename') and rospy.has_param('~distance_waypoints') and rospy.has_param('~world_frame') and rospy.has_param('~add_x_distance') and rospy.has_param('~add_y_distance') and rospy.has_param('~add_z_distance') and rospy.has_param('~tow_width') and rospy.has_param('~tow_thickness'):
            self.config.world_frame = rospy.get_param('~world_frame')
            self.config.course_filename = rospy.get_param('~course_filename')
            self.config.distance_waypoints = rospy.get_param(
                '~distance_waypoints')
            self.config.add_x_distance = rospy.get_param('~add_x_distance')
            self.config.add_y_distance = rospy.get_param('~add_y_distance')
            self.config.add_z_distance = rospy.get_param('~add_z_distance')
            self.config.tow_width = rospy.get_param('~tow_width')*0.0254
            self.config.tow_thickness = rospy.get_param(
                '~tow_thickness')*0.0254
            rospy.loginfo('pose_builder_pyton: Loaded Server parameters')
        else:
            rospy.loginfo(
                'pose_builder_python: Failed to load Server parameters')
            sys.exit(-1)

        if self.config.distance_waypoints == 0.0:
            rospy.loginfo(
                'pose_builder_python: Parameter distance_waypoints defined as 0 or in an array.')
            sys.exit(-1)

        return True

    def readfileContent(self, start_filename):
        filename = rospkg.RosPack().get_path('vsl_msgs') + start_filename
        input = np.loadtxt(filename, dtype='f')
        course = CourseClass()
        for i in range(0, len(input)):
            course.x.append(input[i][0])
            course.y.append(input[i][1])
            course.z.append(input[i][2])
        return course

    def buildBSpline(self, u, k, parameter):
        xd = BSpline(u, self.course.x, k)
        yd = BSpline(u, self.course.y, k)
        zd = BSpline(u, self.course.z, k)

        bspline_course = CourseClass()
        bspline_course.x = xd(parameter)
        bspline_course.y = yd(parameter)
        bspline_course.z = zd(parameter)
        return bspline_course

    def computeArcLengthBetweenWaypoints(self, x1, y1, z1, x2, y2, z2):
        arc_length_waypoints = math.sqrt((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)
        return arc_length_waypoints

    def computeArcLengthCourse(self):
        arc_length = 0
        for i in range(1, len(self.course.x)):
            arc_length = arc_length + self.computeArcLengthBetweenWaypoints(self.course.x[i-1], self.course.y[i-1], self.course.z[i-1],
                                                                            self.course.x[i], self.course.y[i], self.course.z[i])
        return arc_length

    def buildDerivativeBSpline(self, u, k, parameter, order):
        xd = BSpline(u, self.course.x, k)
        yd = BSpline(u, self.course.y, k)
        zd = BSpline(u, self.course.z, k)

        deriv_xd = xd.derivative(order)
        deriv_yd = yd.derivative(order)
        deriv_zd = zd.derivative(order)

        deriv_bspline_course = CourseClass()
        deriv_bspline_course.x = deriv_xd(parameter)
        deriv_bspline_course.y = deriv_yd(parameter)
        deriv_bspline_course.z = deriv_zd(parameter)
        return deriv_bspline_course

    def buildVectorNorm(self, x, y, z):
        vector = CourseClass()
        vector_norm = math.sqrt(x**2+y**2+z**2)
        vector.x.append(x/vector_norm)
        vector.y.append(y/vector_norm)
        vector.z.append(z/vector_norm)
        return vector

    def buildVectorNormfromClass(self, course_vector):
        vector = CourseClass()
        course_class = CourseClass()
        for i in range(0, len(course_vector.x)):
            course_class = self.buildVectorNorm(
                course_vector.x[i], course_vector.y[i], course_vector.z[i])
            vector.x.append(course_class.x[0])
            vector.y.append(course_class.y[0])
            vector.z.append(course_class.z[0])
        return vector

    def getFrenetSerretVectors(self, u, k, parameter):
        deriv1_bspline_course_extrapolated = self.buildDerivativeBSpline(
            u, k, parameter, 1)
        deriv2_bspline_course_extrapolated = self.buildDerivativeBSpline(
            u, k, parameter, 2)
        tangent = self.buildVectorNormfromClass(
            deriv1_bspline_course_extrapolated)
        normal = self.buildVectorNormfromClass(
            deriv2_bspline_course_extrapolated)
        binormal = CourseClass()
        for i in range(0, len(tangent.x)):
            binormal_vector = np.cross([tangent.x[i], tangent.y[i], tangent.z[i]], [
                                       normal.x[i], normal.y[i], normal.z[i]])
            binormal_class = self.buildVectorNorm(
                binormal_vector[0], binormal_vector[1], binormal_vector[2])
            # Enforce a binormal with a positive sign in the z direction (pointing upwards)
            if binormal_class.z[0] < 0:
                binormal_class.x[0] *= -1
                binormal_class.y[0] *= -1
                binormal_class.z[0] *= -1
            binormal.x.append(binormal_class.x[0])
            binormal.y.append(binormal_class.y[0])
            binormal.z.append(binormal_class.z[0])
        return tangent, normal, binormal

    def getParallelTransportVectors(self, u, k, parameter):
        deriv1_bspline_course_extrapolated = self.buildDerivativeBSpline(
            u, k, parameter, 1)
        tangent = self.buildVectorNormfromClass(
            deriv1_bspline_course_extrapolated)
        normal = CourseClass()
        binormal = CourseClass()

        # Initial binormal conditions
        init_binormal = np.array([0.0, 0.0, 1.0])
        binormal.x.append(init_binormal[0])
        binormal.y.append(init_binormal[1])
        binormal.z.append(init_binormal[2])

        prev_binormal = init_binormal

        for i in range(1, len(tangent.x)):
            normal_vector = np.cross(
                [tangent.x[i-1], tangent.y[i-1], tangent.z[i-1]], [tangent.x[i], tangent.y[i], tangent.z[i]])
            if np.linalg.norm(normal_vector) < 0.001:
                binormal_vector = prev_binormal
                binormal.x.append(binormal_vector[0])
                binormal.y.append(binormal_vector[1])
                binormal.z.append(binormal_vector[2])
            else:
                normal_class = self.buildVectorNorm(
                    normal_vector[0], normal_vector[1], normal_vector[2])
                theta = np.arccos(np.dot(
                    [tangent.x[i-1], tangent.y[i-1], tangent.z[i-1]], [tangent.x[i], tangent.y[i], tangent.z[i]]))
                rot_vector = np.dot(rotation_matrix(theta, normal_vector), np.array(
                    [prev_binormal[0], prev_binormal[1], prev_binormal[2], 0]))
                binormal_vector = np.array(
                    [rot_vector[0], rot_vector[1], rot_vector[2]])
                binormal.x.append(binormal_vector[0])
                binormal.y.append(binormal_vector[1])
                binormal.z.append(binormal_vector[2])
            prev_binormal = binormal_vector

        # Build normal vector by using the cross product of the tangent with the now known binormal
        for i in range(0, len(tangent.x)):
            normal_vector = np.cross([tangent.x[i], tangent.y[i], tangent.z[i]], [
                                     binormal.x[i], binormal.y[i], binormal.z[i]])
            normal_class = self.buildVectorNorm(
                normal_vector[0], normal_vector[1], normal_vector[2])
            normal.x.append(normal_class.x[0])
            normal.y.append(normal_class.y[0])
            normal.z.append(normal_class.z[0])
        return tangent, normal, binormal

    def introduceSmoothStart(self, i, course, tangent, normal, binormal):
        single_course_pose = Pose()
        smooth_course_approximation = CourseClass()
        course_extension_npoints = 2
        raise_course_npoints = 3

        if i == 0:
            signal = 1
        else:
            signal = -1

        for f in range(1, course_extension_npoints+1):
            smooth_course_approximation.x.append(course.x[i] - signal * (f / course_extension_npoints) * (
                tangent.x[i] / (tangent.x[i] + tangent.y[i])) * self.XY_EXTENSION_DISTANCE)
            smooth_course_approximation.y.append(course.y[i] - signal * (f / course_extension_npoints) * (
                tangent.y[i] / (tangent.x[i] + tangent.y[i])) * self.XY_EXTENSION_DISTANCE)
            smooth_course_approximation.z.append(course.z[i])

        for f in range(1, raise_course_npoints+1):
            smooth_course_approximation.x.append(smooth_course_approximation.x[course_extension_npoints-1] - signal*(
                f / raise_course_npoints) * (tangent.x[i] / (tangent.x[i] + tangent.y[i])) * self.XY_RAISE_DISTANCE)
            smooth_course_approximation.y.append(smooth_course_approximation.y[course_extension_npoints-1] - signal*(
                f / raise_course_npoints) * (tangent.y[i] / (tangent.x[i] + tangent.y[i])) * self.XY_RAISE_DISTANCE)
            smooth_course_approximation.z.append(
                smooth_course_approximation.z[course_extension_npoints-1] + (f / raise_course_npoints) * self.Z_RAISE_DISTANCE)

        ee_x = [-tangent.x[i], -tangent.y[i], -tangent.z[i]]
        ee_y = [-normal.x[i], -normal.y[i], -normal.z[i]]
        ee_z = [-binormal.x[i], -binormal.y[i], -binormal.z[i]]

        rot = np.array(((ee_x[0], ee_y[0],  ee_z[0], 0.0),
                        (ee_x[1],  ee_y[1], ee_z[1], 0.0),
                        (ee_x[2],   ee_y[2],  ee_z[2], 0.0),
                        (0.0,  0.0,  0.0, 1.0)))

        q = quaternion_from_matrix(rot)
        single_course_pose.orientation.x = q[0]
        single_course_pose.orientation.y = q[1]
        single_course_pose.orientation.z = q[2]
        single_course_pose.orientation.w = q[3]

        if single_course_pose.orientation.w < 0.0:
            single_course_pose.orientation.x *= -1
            single_course_pose.orientation.y *= -1
            single_course_pose.orientation.z *= -1
            single_course_pose.orientation.w *= -1

        for f in range(0, course_extension_npoints+raise_course_npoints):
            single_course_pose.position.x = - \
                (smooth_course_approximation.x[f] -
                 self.TABLE_WIDTH - self.TABLE_WIDTH / 2) + self.config.add_x_distance
            single_course_pose.position.y = - \
                (smooth_course_approximation.y[f] -
                 self.TABLE_WIDTH / 2) + self.config.add_y_distance
            single_course_pose.position.z = smooth_course_approximation.z[f] + \
                self.TABLE_HEIGHT + self.APPROACH_TABLE + self.config.add_z_distance

            if i == 0:
                self.course_poses.poses.insert(0, single_course_pose)
            else:
                self.course_poses.poses.append(single_course_pose)
        return True

    def createCourse(self):
        # Starts by reading the file content defined in the launch filed
        self.course = self.readfileContent(self.config.course_filename)
        n_points = len(self.course.x)

        # Computes number of waypoints
        arc_length = self.computeArcLengthCourse()
        n_waypoints = int(arc_length // self.config.distance_waypoints)

        if n_waypoints < 2:
            rospy.loginfo(
                'pose_builder_python: The distance requested between waypoint is large, cannot interpolate course')
            sys.exit(-1)
        else:
            rospy.loginfo('pose_builder_pyton: Trajectory given with %d points it will be interpolated to %d waypoints, with distances of %f m between waypoitns',
                          n_points, n_waypoints, self.config.distance_waypoints)

        # Builds curve degree and knot vector, should pass by all points
        parameter = np.linspace(0, 1, n_waypoints)
        k = len(self.course.x)-1
        u = []
        for i in range(0, 2*k+2):
            if i < k+1:
                u.append(0)
            else:
                u.append(1)

        # Builds BSpline of the course assuming a constant parameter
        bspline_course_extrapolated = self.buildBSpline(u, k, parameter)

        # Compute tangent, normal and binormal using the definition of Frenet Serret or Parallel Transport
        # tangent, normal, binormal = self.getFrenetSerretVectors(u, k, parameter)
        tangent, normal, binormal = self.getParallelTransportVectors(
            u, k, parameter)

        # Construct transformation matrix and store in a geometry_msgs type
        self.course_poses.header.frame_id = self.config.world_frame

        for i in range(0, n_waypoints):
            single_course_pose = Pose()
            # Modify axes direction in relation to the base frame
            ee_x = [-tangent.x[i], -tangent.y[i], -tangent.z[i]]
            ee_y = [-normal.x[i], -normal.y[i], -normal.z[i]]
            ee_z = [-binormal.x[i], -binormal.y[i], -binormal.z[i]]

            rot = np.array(((ee_x[0], ee_y[0],  ee_z[0], 0.0),
                            (ee_x[1],  ee_y[1], ee_z[1], 0.0),
                            (ee_x[2],   ee_y[2],  ee_z[2], 0.0),
                            (0.0,  0.0,  0.0, 1.0)))

            q = quaternion_from_matrix(rot)
            single_course_pose.orientation.x = q[0]
            single_course_pose.orientation.y = q[1]
            single_course_pose.orientation.z = q[2]
            single_course_pose.orientation.w = q[3]

            if single_course_pose.orientation.w < 0.0:
                single_course_pose.orientation.x *= -1
                single_course_pose.orientation.y *= -1
                single_course_pose.orientation.z *= -1
                single_course_pose.orientation.w *= -1

            single_course_pose.position.x = - \
                (bspline_course_extrapolated.x[i] -
                 self.TABLE_WIDTH - self.TABLE_WIDTH / 2) + self.config.add_x_distance
            single_course_pose.position.y = - \
                (bspline_course_extrapolated.y[i] -
                 self.TABLE_WIDTH / 2) + self.config.add_y_distance
            single_course_pose.position.z = bspline_course_extrapolated.z[i] + \
                self.TABLE_HEIGHT + self.APPROACH_TABLE + self.config.add_z_distance
            self.course_poses.poses.append(single_course_pose)

        self.introduceSmoothStart(
            0, bspline_course_extrapolated, tangent, normal, binormal)
        self.introduceSmoothStart(
            n_waypoints-1, bspline_course_extrapolated, tangent, normal, binormal)

        pose_builder_server = rospy.Service(
            self.POSE_BUILDER_SERVICE, PoseBuilder, self.serviceCallback, 1000)
        return True

    def serviceCallback(self, req):
        res = PoseBuilderResponse()
        res.single_course_poses = self.course_poses
        return res


if __name__ == "__main__":
    rospy.init_node('pose_builder_python')
    try:
        ns = PoseBuilderPython()
        ns.initServer()
        ns.createCourse()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    # def findInterpolationIndexes(self, bspline_course):
    #     distance_requested = self.distance_waypoints
    #     arc_length = computeArcLengthCourse(self)
    #     n_waypoints = arc_length // distance_requested

    #     if n_waypoints < 2:
    #         rospy.loginfo('pose_builder_python: The distance requested between waypoint is large, cannot interpolate course')
    #         sys.exit(-1)

    #     if arc_length % distance_requested != 0.0:
    #         extend_inter = True
    #     else:
    #         extend_inter = False

    #     distance = 0.0
    #     position = []
    #     position_index = 0
    #     position.append(position_index)

    #     for n in range(1, n_waypoints-1):
    #         percentage = 0.01
    #         for i in range(position_index, len(bspline_course.x)-1):
    #             distance = distance + computeArcLengthBetweenWaypoints(bspline_course.x[i],bspline_course.y[i],bspline_course.z[i],
    #                                                                     bspline_course.x[i+1],bspline_course.y[i+1],bspline_course.z[i+1])
    #             if (distance >= (1-percentage)*distance_requested and distance <= (1+percentage)*distance_requested):
    #                 if distance > distance_requested:
    #                     percentage = distance / distance_requested-1
    #                 else:
    #                     percentage = 1-distance / distance_requested

    #                 position_index = i
    #         position.append(position_index)
    #         distance = 0.0
    #     position.append(len(bspline_course.x)-1)
    #     return position


# def compute_radius2D(dp, ddp):
#     radius = []
#     for i in range(0, len(dp.x)):
#         num = math.sqrt(dp.x[i]**2+dp.y[i]**2)**3
#         denom = dp.x[i]*ddp.y[i]-dp.y[i]*ddp.x[i]
#         radius.append(abs(num/denom))
#     return radius

# def compute_radius3D(dp, ddp):
#     radius = []
#     for i in range(0, len(dp.x)):
#         num = math.sqrt(dp.x[i]**2+dp.y[i]**2+dp.z[i]**2)**3
#         denom = math.sqrt((dp.y[i]*ddp.z[i]-ddp.y[i]*dp.z[i])**2+(dp.x[i]*ddp.z[i]-ddp.x[i]*dp.z[i])**2+(dp.x[i]*ddp.y[i]-dp.y[i]*ddp.x[i])**2)
#         radius.append(abs(num/denom))
#     return radius
