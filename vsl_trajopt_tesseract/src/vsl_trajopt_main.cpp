#include <vsl_trajopt_planner.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vsl_trajopt_main");

  vsl_motion_planner::VSLTrajoptPlanner planner;
  planner.run();

}
