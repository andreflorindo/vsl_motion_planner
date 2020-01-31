/* Author: Andre Florindo*/

/* Goal: With the cartesian trajectory execute Descartes planning algoritm (PlanPath) and 
        obtain the respective joint space trajectory (GetPath)
*/

#include <vsl_descartes_moveit_planner.h>

namespace vsl_motion_planning
{

void VSLDescartesMoveitPlanner::planPath(std::vector<descartes_core::TrajectoryPtPtr> &input_traj,
                          std::vector<descartes_core::TrajectoryPtPtr> &output_path)
{
  //modifying start and end points such that are close to "seed_pose" -> Pose with joint values close to the desired start of the robot path
  std::vector<double> start_pose, end_pose;
  if (input_traj.front()->getClosestJointPose(config_.seed_pose, *robot_model_ptr_, start_pose) &&
      input_traj.back()->getClosestJointPose(config_.seed_pose, *robot_model_ptr_, end_pose))
  {
    ROS_INFO_STREAM("Setting trajectory start and end to JointTrajectoryPts");

    // Creating Start JointTrajectoryPt from start joint pose
    descartes_core::TrajectoryPtPtr start_joint_point = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::JointTrajectoryPt(start_pose));

    // Creating End JointTrajectoryPt from end joint pose
    descartes_core::TrajectoryPtPtr end_joint_point = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::JointTrajectoryPt(end_pose));

    // Modifying start and end of the trajectory, with a selected joint pose
    // Although not necessary, it reduces the time to get a solution and unexpected outcomes
    // With the algorith these will be later replaced
    input_traj[0] = start_joint_point;
    input_traj[input_traj.size() - 1] = end_joint_point;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to find closest joint pose to seed pose at the start or end of trajectory");
    exit(-1);
  }

  // Plan a robot path from the trajectory
  bool succeeded = planner_.planPath(input_traj);

  if (succeeded)
  {
    ROS_INFO_STREAM("Valid path was found");
  }
  else
  {
    ROS_ERROR_STREAM("Could not solve for a valid path");
    exit(-1);
  }

  // Retrieve the planned robot path.
  succeeded = planner_.getPath(output_path);

  if (!succeeded || output_path.empty())
  {
    ROS_ERROR_STREAM("Failed to retrieve robot path");
  }

  ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

} // namespace vsl_motion_planning
