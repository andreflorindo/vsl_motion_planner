#include <vsl_trajopt_planner.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vsl_trajopt_main");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  bool plotting = true;
  bool rviz = true;
  vsl_motion_planner::VSLTrajoptPlannerConfiguration config;

  if (pnh.param("plotting", plotting, plotting) &&
      pnh.param("rviz", rviz, rviz) &&
      pnh.getParam("group_name", config.group_name) &&
      pnh.getParam("tip_link", config.tip_link) &&
      pnh.getParam("base_link", config.base_link) &&
      pnh.getParam("world_frame", config.world_frame))
  {
    ROS_INFO_STREAM("Loaded application parameters");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load application parameters");
    exit(-1);
  }

  vsl_motion_planner::VSLTrajoptPlanner planner(nh, plotting, rviz, config);
  planner.run();
}
