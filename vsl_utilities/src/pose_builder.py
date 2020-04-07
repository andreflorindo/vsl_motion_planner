#! /usr/bin/env python
# Author: Andre Florindo
# Requires Python 2.7

# This node read a txt file containing the course path and return the transformation of each waypoint
# The structure of the txt file sould be the following:
# Layer L1
#
# Course C1
#
# X1 Y1 Z1
# X2 Y2 Z2
# ...
#
# Course C2

# Python
from __future__ import division
import sys
import numpy as np
import math
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline, splprep, splev
# from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_from_matrix, rotation_matrix
import re

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
        self.smooth_start = bool
        self.interpolate = bool


class PoseBuilderPython:
    TABLE_HEIGHT = 0.78
    TABLE_WIDTH = 1.2
    APPROACH_TABLE = 0.002
    XY_EXTENSION_DISTANCE = 0.02  # meters
    XY_RAISE_DISTANCE = 0.10  # meters
    ANGLE_RAISE = 10  # degrees
    Z_RAISE_DISTANCE = XY_RAISE_DISTANCE*math.tan(ANGLE_RAISE*math.pi/180)
    SERVER_TIMEOUT = 5.0  # seconds
    POSE_BUILDER_SERVICE = "single_course"
    POSE_BUILDER_PUBLISHER = "single_course_poses"

    def __init__(self):
        self.config = PoseBuilderConfiguration()
        self.course = CourseClass()
        self.course_poses = PoseArray()
        self.req_layer = []
        self.req_course = []
        self.pose_builder_server = []
        self.n_waypoints = []
        # self.pose_builder_publisher = []

    def initServer(self):
        if rospy.has_param('~course_filename') and rospy.has_param('~distance_waypoints') and rospy.has_param('~world_frame') and rospy.has_param('~add_x_distance') and rospy.has_param('~add_y_distance') and rospy.has_param('~add_z_distance') and rospy.has_param('~tow_width') and rospy.has_param('~tow_thickness') and rospy.has_param('~smooth_start'):
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
            self.config.smooth_start = rospy.get_param(
                '~smooth_start')
            self.config.interpolate = rospy.get_param(
                '~interpolate')

            rospy.loginfo('pose_builder_python: Loaded Server parameters')
        else:
            rospy.logerr(
                'pose_builder_python: Failed to load Server parameters')
            sys.exit(-1)

        if self.config.distance_waypoints == 0.0:
            rospy.logerr(
                'pose_builder_python: Parameter distance_waypoints defined as 0')
            sys.exit(-1)
        self.pose_builder_server = rospy.Service(
            self.POSE_BUILDER_SERVICE, PoseBuilder, self.serviceCallback, 1)
        # self.pose_builder_publisher = rospy.Publisher(self.POSE_BUILDER_PUBLISHER, PoseArray, queue_size=1)
        return True

    def serviceCallback(self, req):
        self.req_layer = req.num_layer
        self.req_course = req.num_course
        res = PoseBuilderResponse()
        self.createCourse()
        res.single_course_poses = self.course_poses
        # self.pose_builder_publisher.publish(self.course_poses)
        return res

    def countTotalNumberOfLayersAndCourses(self, start_filename):
        filename = rospkg.RosPack().get_path('vsl_msgs') + start_filename
        infile = open(filename, 'r')
        num_layers = 0
        num_courses = []
        file_position = -1
        for line_layer in infile:
            file_position += 1
            search_layer = re.search(r'Layer', line_layer)
            if search_layer:
                num_layers += 1
                num_courses.append(0)
                for line_course in infile:
                    file_position += 1
                    search_course = re.search(r'Course', line_course)
                    if search_course:
                        num_courses[num_layers-1] += 1
                    search_next_layer = re.search(r'Layer', line_course)
                    if search_next_layer:
                        offset = file_position*12
                        infile.seek(offset, 0)
                        break
        infile.close()
        total_num_courses = 0
        for i in range(0, num_layers):
            total_num_courses += num_courses[i]
        rospy.loginfo('pose_builder_python: Laminate contains %d layers, with and a total of %d fiber courses',
                      num_layers, total_num_courses)
        return num_layers, num_courses

    def readfileContent(self, start_filename):
        # filename = rospkg.RosPack().get_path('vsl_msgs') + start_filename
        # input = np.loadtxt(filename, dtype='f')
        # course = CourseClass()
        # for i in range(0, len(input)):
        #     course.x.append(input[i][0])
        #     course.y.append(input[i][1])
        #     course.z.append(input[i][2])
        # return course

        filename = rospkg.RosPack().get_path('vsl_msgs') + start_filename
        course = CourseClass()
        infile = open(filename, 'r')
        num_layer = 0
        num_course = 0
        for line_layer in infile:
            search_layer = re.search(r'Layer', line_layer)
            if search_layer:
                num_layer += 1
                if num_layer == self.req_layer:
                    for line_course in infile:
                        search_course = re.search(r'Course', line_course)
                        if search_course:
                            num_course += 1
                            if num_course == self.req_course:
                                line_course = next(infile)
                                line_course = next(infile)
                                input = re.findall(
                                    r"-?\ *[0-9]+\.?[0-9]*(?:[Ee]\ *-?\ *[0-9]+)?", line_course)
                                while len(input) != 0:
                                    course.x.append(float(input[0]))
                                    course.y.append(float(input[1]))
                                    course.z.append(float(input[2]))
                                    line_course = next(infile)
                                    input = re.findall(
                                        r"-?\ *[0-9]+\.?[0-9]*(?:[Ee]\ *-?\ *[0-9]+)?", line_course)
        infile.close()
        return course

    def buildBSpline(self, u, k):
        xd = BSpline(u, self.course.x, k)
        yd = BSpline(u, self.course.y, k)
        zd = BSpline(u, self.course.z, k)
        
        parameter = np.linspace(0, 1, self.n_waypoints)

        bspline_course = CourseClass()
        bspline_course.x = xd(parameter)
        bspline_course.y = yd(parameter)
        bspline_course.z = zd(parameter)
        return bspline_course


    def buildInterpolatedBSpline(self):
        tck, u = splprep([self.course.x,self.course.y,self.course.z], k=3, s=0.000001)
        u_new = np.linspace(u.min(), u.max(), self.n_waypoints)
        bspline_course = CourseClass()
        bspline_course.x, bspline_course.y, bspline_course.z = splev(u_new, tck, der=0)
   
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

    def buildDerivativeBSpline(self, u, k, order):
        xd = BSpline(u, self.course.x, k)
        yd = BSpline(u, self.course.y, k)
        zd = BSpline(u, self.course.z, k)

        deriv_xd = xd.derivative(order)
        deriv_yd = yd.derivative(order)
        deriv_zd = zd.derivative(order)
        
        parameter = np.linspace(0, 1, self.n_waypoints)

        deriv_bspline_course = CourseClass()
        deriv_bspline_course.x = deriv_xd(parameter)
        deriv_bspline_course.y = deriv_yd(parameter)
        deriv_bspline_course.z = deriv_zd(parameter)
        return deriv_bspline_course

    def buildDerivativeInterpolatedBSpline(self, order):
        tck, u = splprep([self.course.x,self.course.y,self.course.z], k=3, s=0.000001)
        u_new = np.linspace(u.min(), u.max(), self.n_waypoints)
        deriv_bspline_course = CourseClass()
        deriv_bspline_course.x, deriv_bspline_course.y, deriv_bspline_course.z = splev(u_new, tck, der=order)
    
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

    def getFrenetSerretVectors(self, u, k):

        if (self.config.interpolate):
            deriv1_bspline_course_extrapolated = self.buildDerivativeInterpolatedBSpline(1)
            deriv2_bspline_course_extrapolated = self.buildDerivativeInterpolatedBSpline(2)
        else:
            deriv1_bspline_course_extrapolated = self.buildDerivativeBSpline(u, k, 1)
            deriv2_bspline_course_extrapolated = self.buildDerivativeBSpline(u, k, 2)

        tangent = self.buildVectorNormfromClass(
            deriv1_bspline_course_extrapolated)
        aux_normal = self.buildVectorNormfromClass(
            deriv2_bspline_course_extrapolated)

        normal = CourseClass()
        binormal = CourseClass()
        for i in range(0, len(tangent.x)):
            binormal_vector = np.cross([tangent.x[i], tangent.y[i], tangent.z[i]], [
                                       aux_normal.x[i], aux_normal.y[i], aux_normal.z[i]])
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

        for i in range(0, len(tangent.x)):
            normal_vector = np.cross([tangent.x[i], tangent.y[i], tangent.z[i]], [
                                     binormal.x[i], binormal.y[i], binormal.z[i]])
            normal_class = self.buildVectorNorm(
                normal_vector[0], normal_vector[1], normal_vector[2])
            normal.x.append(normal_class.x[0])
            normal.y.append(normal_class.y[0])
            normal.z.append(normal_class.z[0])

        return tangent, normal, binormal

    def getParallelTransportVectors(self, u, k):
        
        if (self.config.interpolate):
            deriv1_bspline_course_extrapolated = self.buildDerivativeInterpolatedBSpline(1)
        else:
            deriv1_bspline_course_extrapolated = self.buildDerivativeBSpline(u, k, 1)

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
        smooth_course_approximation = CourseClass()
        # course_extension_npoints = int(self.XY_EXTENSION_DISTANCE // self.config.distance_waypoints)
        # raise_course_npoints = int((self.XY_RAISE_DISTANCE/math.cos(self.ANGLE_RAISE*math.pi/180)) // self.config.distance_waypoints)
        course_extension_npoints = 1
        raise_course_npoints = 10

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
            #smooth_course_approximation.z.append(course.z[i])

        for f in range(0, course_extension_npoints+raise_course_npoints):
            single_course_pose = Pose()
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

            single_course_pose.position.x = smooth_course_approximation.x[f] + \
                self.TABLE_WIDTH + self.config.add_x_distance
            single_course_pose.position.y = smooth_course_approximation.y[f] + self.config.tow_width*(
                self.req_course-1) + self.config.add_y_distance
            single_course_pose.position.z = smooth_course_approximation.z[f] + self.TABLE_HEIGHT + \
                self.APPROACH_TABLE + self.config.tow_thickness * \
                (self.req_layer-1) + self.config.add_z_distance

            if i == 0:
                self.course_poses.poses.insert(0, single_course_pose)
            else:
                self.course_poses.poses.append(single_course_pose)

        if i == 0:
            rospy.loginfo(
                'pose_builder_python: Added %d waypoints to allow for a smoother start', course_extension_npoints + raise_course_npoints)
        else:
            rospy.loginfo(
                'pose_builder_python: Added %d waypoints to allow for a smoother end', course_extension_npoints + raise_course_npoints)

        return True

    def createCourse(self):
        # Starts by reading the file content defined in the launch filed
        n_layers, vector_n_courses = self.countTotalNumberOfLayersAndCourses(
            self.config.course_filename)

        if (n_layers >= self.req_layer):
            if (vector_n_courses[self.req_layer-1] >= self.req_course):
                rospy.loginfo(
                    'pose_builder_python: Computing transformations for waypoints of course %d of layer %d', self.req_course, self.req_layer)
            else:
                rospy.logerr(
                    'pose_builder_python: Course %d of Layer %d cannot be found in the file', self.req_course, self.req_layer)
                sys.exit(-1)
        else:
            rospy.logerr('pose_builder_python: Layer %d cannot be found in the file',
                         self.req_layer, self.req_course)
            sys.exit(-1)

        self.course = self.readfileContent(self.config.course_filename)
        n_points = len(self.course.x)

        # Computes number of waypoints
        arc_length = self.computeArcLengthCourse()
        self.n_waypoints = int(arc_length // self.config.distance_waypoints)

        if self.n_waypoints < 2:
            rospy.logerr(
                'pose_builder_python: The distance requested between waypoint is large, cannot interpolate course')
            sys.exit(-1)
        elif self.n_waypoints > 10000:
            rospy.logerr(
                'pose_builder_python: The distance requested between waypoint is small, requested to many waypoints')
            sys.exit(-1)
        else:
            rospy.loginfo('pose_builder_python: Trajectory given with %d points it will be interpolated to %d waypoints, with distances of %f m between waypoitns',
                          n_points, self.n_waypoints, self.config.distance_waypoints)


        # Builds curve degree and knot vector, should pass by all points. 
        # Only required if self.config.interpolate is false
        k = len(self.course.x)-1
        u = []
        for i in range(0, 2*k+2):
            if i < k+1:
                u.append(0)
            else:
                u.append(1)

       
        if (self.config.interpolate):
            # Uses waypoints to build the bspline
            bspline_course_extrapolated = self.buildInterpolatedBSpline()
        else: 
            # Uses control points to build the bspline
            bspline_course_extrapolated = self.buildBSpline(u, k) 

        # Compute tangent, normal and binormal using the definition of Frenet Serret or Parallel Transport
        # tangent, normal, binormal = self.getFrenetSerretVectors(u, k)
        
        tangent, normal, binormal = self.getParallelTransportVectors(u, k)

        # Construct transformation matrix and store in a geometry_msgs type
        self.course_poses.header.stamp = rospy.Time.now()
        self.course_poses.header.frame_id = self.config.world_frame

        for i in range(0, self.n_waypoints):
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

            # single_course_pose.position.x = - \
            #     (bspline_course_extrapolated.x[i] -
            #      self.TABLE_WIDTH - self.TABLE_WIDTH / 2) + self.config.add_x_distance
            # single_course_pose.position.y = - \
            #     (bspline_course_extrapolated.y[i] -
            #      self.TABLE_WIDTH / 2) + self.config.tow_width*(self.req_course-1) + self.config.add_y_distance
            # single_course_pose.position.z = bspline_course_extrapolated.z[i] + \
            #     self.TABLE_HEIGHT + self.APPROACH_TABLE + self.config.tow_thickness*(self.req_layer-1) + self.config.add_z_distance
            # self.course_poses.poses.append(single_course_pose)

            single_course_pose.position.x = bspline_course_extrapolated.x[i] + \
                self.TABLE_WIDTH + self.config.add_x_distance
            single_course_pose.position.y = bspline_course_extrapolated.y[i] + self.config.tow_width*(
                self.req_course-1) + self.config.add_y_distance
            single_course_pose.position.z = bspline_course_extrapolated.z[i] + self.TABLE_HEIGHT + \
                self.APPROACH_TABLE + self.config.tow_thickness * \
                (self.req_layer-1) + self.config.add_z_distance
            self.course_poses.poses.append(single_course_pose)

        if(self.config.smooth_start):
            self.introduceSmoothStart(
                0, bspline_course_extrapolated, tangent, normal, binormal)
            self.introduceSmoothStart(
                self.n_waypoints-1, bspline_course_extrapolated, tangent, normal, binormal)

        return True


if __name__ == "__main__":
    rospy.init_node('pose_builder_python')
    try:
        ns = PoseBuilderPython()
        ns.initServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    # def findInterpolationIndexes(self, bspline_course):
    #     distance_requested = self.distance_waypoints
    #     arc_length = computeArcLengthCourse(self)
    #     self.n_waypoints = arc_length // distance_requested

    #     if self.n_waypoints < 2:
    #         rospy.logerr('pose_builder_python: The distance requested between waypoint is large, cannot interpolate course')
    #         sys.exit(-1)

    #     if arc_length % distance_requested != 0.0:
    #         extend_inter = True
    #     else:
    #         extend_inter = False

    #     distance = 0.0
    #     position = []
    #     position_index = 0
    #     position.append(position_index)

    #     for n in range(1, self.n_waypoints-1):
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
