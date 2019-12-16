#include <vsl_trajopt_planner.h>

using namespace vsl_motion_planner;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vsl_trajopt_planner");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);

  PickAndPlaceExample example(nh, plotting, rviz, steps, write_to_file);
  example.run();

}
