/* Author: Andre Florindo*/

/* Goal: Creates a RobotTrajectory with Descartes Trajectory and executes path
*/

#include <vsl_descartes_moveit_planner.h>

namespace vsl_motion_planning
{

void VSLDescartesMoveitPlanner::runPath(const std::vector<descartes_core::TrajectoryPtPtr> &path)
{

  // moving to "above-table" configuration
  // creating move group to move the arm in free space
  moveit::planning_interface::MoveGroupInterface move_group(config_.group_name);
  move_group.setPlannerId(PLANNER_ID); //RRTConnect
  move_group.setPlanningTime(10.0f);
  move_group.setMaxVelocityScalingFactor(config_.max_joint_speed_scaling_between_traj);

  // setting above-table position as target
  if (!move_group.setNamedTarget(HOME_POSITION_NAME))
  {
    ROS_ERROR_STREAM("Failed to set home '" << HOME_POSITION_NAME << "' position");
    exit(-1);
  }

  moveit_msgs::MoveItErrorCodes result = move_group.move();
  if (result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to move to " << HOME_POSITION_NAME << " position");
    exit(-1);
  }
  else
  {
    ROS_INFO_STREAM("Robot reached home position");
  }

  // creating goal joint pose to start of the path
  std::vector<double> seed_pose(robot_model_ptr_->getDOF());
  std::vector<double> start_pose;

  descartes_core::TrajectoryPtPtr first_point_ptr = path[0];
  first_point_ptr->getNominalJointPose(seed_pose, *robot_model_ptr_, start_pose);

  move_group.setJointValueTarget(start_pose);

  result = move_group.move();
  if (result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Move to start joint pose failed");
    exit(-1);
  }
  else
  {
    ROS_INFO_STREAM("Robot reached start position");
  }

  // creating Moveit trajectory from Descartes Trajectory
  moveit_msgs::RobotTrajectory moveit_traj;
  fromDescartesToMoveitTrajectory(path, moveit_traj.joint_trajectory);

  /////////////////////////////////////Time parameterization Descartes ///////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  // addTimeParameterizationToDescartes(moveit_traj);
  
  moveit_msgs::ExecuteTrajectoryGoal goal;
  goal.trajectory = moveit_traj;

  ROS_INFO_STREAM("Robot path sent for execution");
  if (moveit_run_path_client_ptr_->sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO_STREAM("Robot path execution completed");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to run robot path with error " << *moveit_run_path_client_ptr_->getResult());
    exit(-1);
  }

  ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

void VSLDescartesMoveitPlanner::fromDescartesToMoveitTrajectory(const std::vector<descartes_core::TrajectoryPtPtr> &input_traj,
                                                 trajectory_msgs::JointTrajectory &traj)
{
  //  // Fill out information about our trajectory
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = config_.world_frame;
  traj.joint_names = config_.joint_names;

  descartes_utilities::toRosJointPoints(*robot_model_ptr_, input_traj, VELOCITY_DESCARTES, traj.points);
  addVel(traj);
  addAcc(traj);
}

void VSLDescartesMoveitPlanner::addVel(trajectory_msgs::JointTrajectory &traj) //Velocity of the joints
{
  if (traj.points.size() < 3)
    return;

  auto n_joints = traj.points.front().positions.size();

  for (auto i = 0; i < n_joints; ++i)
  {
    traj.points[0].velocities[i] = 0.0f;
    traj.points[traj.points.size() - 1].velocities[i] = 0.0f;
    for (auto j = 1; j < traj.points.size() - 1; j++)
    {
      // For each point in a given joint
      //Finite difference, first order, central. Gives the average velocity, not conservative
      double delta_theta = -traj.points[j - 1].positions[i] + traj.points[j + 1].positions[i];
      double delta_time = -traj.points[j - 1].time_from_start.toSec() + traj.points[j + 1].time_from_start.toSec();
      double v = delta_theta / delta_time;
      traj.points[j].velocities[i] = v;
    }
  }
}

void VSLDescartesMoveitPlanner::addAcc(trajectory_msgs::JointTrajectory &traj) //Velocity of the joints
{
    if (traj.points.size() < 3)
        return;

    auto n_joints = traj.points.front().positions.size();

    for (auto i = 0; i < n_joints; ++i)
    {
        traj.points[0].accelerations[i] = 0.0f;      // <- Incorrect!!!! TODO
        traj.points[traj.points.size() - 1].accelerations[i] = 0.0f;

        for (auto j = 1; j < traj.points.size()-1; j++)
        {
            // For each point in a given joint
            //Finite difference, first order, central. Gives the average velocity, not conservative
            double delta_velocity = -traj.points[j - 1].velocities[i] + traj.points[j + 1].velocities[i];
            double delta_time = -traj.points[j - 1].time_from_start.toSec() + traj.points[j + 1].time_from_start.toSec();
            double a = delta_velocity / delta_time;
            traj.points[j].accelerations[i] = a;
        }
    }
}

void VSLDescartesMoveitPlanner::addTimeParameterizationToDescartes(moveit_msgs::RobotTrajectory &traj)
{
  robot_trajectory::RobotTrajectory robot_trajectory(robot_model_loader_->getModel(), config_.group_name);

  robot_trajectory.setRobotTrajectoryMsg(*kinematic_state_,traj);
  //time_parameterization_.computeTimeStamps(robot_trajectory, 0.05, 1);
  
  vsl_motion_planning::ConstEESpeedTimeParameterization designed_time_parameterization;
  designed_time_parameterization.computeTimeStamps(robot_trajectory, config_.tip_link, config_.ee_speed, 1, 1);

  robot_trajectory.getRobotTrajectoryMsg(traj);
}

// void VSLDescartesMoveitPlanner::addVel(trajectory_msgs::JointTrajectory &traj) //Velocity of the joints
// {
//   if (traj.points.size() < 3)
//     return;

//   auto n_joints = traj.points.front().positions.size();

//   for (auto i = 0; i < n_joints; ++i)
//   {
//     traj.points[0].velocities[i] = 0.0f;
//     //traj.points[traj.points.size()-1].velocities[i] = 0.0f;

//     for (auto j = 1; j < traj.points.size()-1; j++)
//     {
//       // For each point in a given joint
//       //Finite difference, first order, regressive
//       double delta_theta = traj.points[j].positions[i] - traj.points[j - 1].positions[i];
//       double delta_time = traj.points[j].time_from_start.toSec() - traj.points[j - 1].time_from_start.toSec();
//       double v = delta_theta / delta_time;
//       traj.points[j].velocities[i] = v;
//     }

//   }
// }

// void VSLDescartesMoveitPlanner::addAcc(trajectory_msgs::JointTrajectory &traj) //Velocity of the joints
// {
//   if (traj.points.size() < 3)
//     return;

//   auto n_joints = traj.points.front().positions.size();

//   for (auto i = 0; i < n_joints; ++i)
//   {
//     traj.points[0].accelerations[i] = 0.0f;
//     //traj.points[traj.points.size()-1].velocities[i] = 0.0f;

//     for (auto j = 1; j < traj.points.size(); j++)
//     {
//       // For each point in a given joint
//       //Finite difference, first order, regressive
//       double delta_velocity = traj.points[j].velocities[i] - traj.points[j - 1].velocities[i];
//       double delta_time = traj.points[j].time_from_start.toSec() - traj.points[j - 1].time_from_start.toSec();
//       double a = delta_velocity / delta_time;
//       traj.points[j].accelerations[i] = a;
//     }

//   }
} // namespace vsl_motion_planning
