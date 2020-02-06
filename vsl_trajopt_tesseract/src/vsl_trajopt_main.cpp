#include <vsl_trajopt_planner.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vsl_trajopt_main");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  vsl_motion_planner::VSLTrajoptPlanner planner;
  planner.run();
  
  spinner.stop();
  return 0;
}
