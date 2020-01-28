#include <vsl_descartes_moveit_planner.h>

/* INIT ROS
  Goal:
    - Load missing application parameters from the ros parameter server.
    - Use a private NodeHandle in order to load parameters defined in the node's namespace.
    - Create a ros service client that will be used to send a robot path for execution.
*/

namespace vsl_motion_planning
{

VSLDescartesMoveitPlanner::VSLDescartesMoveitPlanner() {}
VSLDescartesMoveitPlanner::~VSLDescartesMoveitPlanner() {}

void VSLDescartesMoveitPlanner::initRos()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("group_name", config_.group_name) &&
        ph.getParam("tip_link", config_.tip_link) &&
        ph.getParam("base_link", config_.base_link) &&
        ph.getParam("world_frame", config_.world_frame) &&
        ph.getParam("trajectory/seed_pose", config_.seed_pose) &&
        ph.getParam("max_joint_speed_scaling_between_traj", config_.max_joint_speed_scaling_between_traj) &&
        ph.getParam("ee_speed", config_.ee_speed) &&
        nh.getParam("controller_joint_names", config_.joint_names))
    {
        ROS_INFO_STREAM("Loaded application parameters");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load application parameters");
        exit(-1);
    }

    typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> client_type;
    moveit_run_path_client_ptr_ = std::make_shared<client_type>(EXECUTE_TRAJECTORY_ACTION, true);

    // Establishing connection to server
    if (moveit_run_path_client_ptr_->waitForServer(ros::Duration(SERVER_TIMEOUT)))
    {
        ROS_INFO_STREAM("Connected to '" << EXECUTE_TRAJECTORY_ACTION << "' action");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to connect to '" << EXECUTE_TRAJECTORY_ACTION << "' action");
        exit(-1);
    }

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

} // namespace vsl_motion_planning
