#include <vsl_ompl_moveit_planner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vsl_ompl_moveit_main");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    vsl_motion_planning::VSLOMPLMoveitPlanner planner;

    planner.initOmpl();

    std::vector<geometry_msgs::Pose> waypoints;
    planner.getCourse(waypoints);

    planner.createMotionPlanRequest(waypoints);

    return 0;
}