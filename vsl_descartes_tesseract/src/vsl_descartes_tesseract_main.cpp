#include <vsl_descartes_tesseract_planner.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vsl_descartes_tesseract_main");

  vsl_motion_planner::VSLDescartesTesseractPlanner planner;
  planner.run();

  ROS_INFO("Done");
  ros::spin();
  return 0;
}
