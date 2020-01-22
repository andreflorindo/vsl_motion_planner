#include <vsl_descartes_moveit_planner.h>

namespace vsl_motion_planning
{
bool VSLDescartesMoveitPlanner::getCourse(EigenSTL::vector_Isometry3d &poses)
{
    // Initialize Service client
    if (ros::service::waitForService(POSE_BUILDER_SERVICE, ros::Duration(SERVER_TIMEOUT)))
    {
        ROS_INFO_STREAM("Connected to '" << POSE_BUILDER_SERVICE << "' service");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to connect to '" << POSE_BUILDER_SERVICE << "' service");
        exit(-1);
    }

    pose_builder_client_ = nh_.serviceClient<vsl_msgs::PoseBuilder>(POSE_BUILDER_SERVICE);
    vsl_msgs::PoseBuilder srv;
    // srv.request.num_layer = num_layer;
    // srv.request.num_course = num_course;
    // ROS_INFO_STREAM("Requesting pose in base frame: " << num_layer);

    if (!pose_builder_client_.call(srv))
    {
        ROS_ERROR_STREAM("Failed to call '" << POSE_BUILDER_SERVICE << "' service");
        exit(-1);
    }

    // Modify the single_pose type from PoseArray to Isometry3d
    Eigen::Isometry3d single_pose;
    poses.reserve(srv.response.single_course_poses.poses.size());

    for (unsigned int i = 0; i < srv.response.single_course_poses.poses.size(); i++)
    {
        tf::poseMsgToEigen(srv.response.single_course_poses.poses[i], single_pose);
        poses.emplace_back(single_pose);
    }
    
    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}
} // namespace vsl_motion_planning

