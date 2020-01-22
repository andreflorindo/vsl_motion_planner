#include <vsl_descartes_moveit_planner.h>

/* INIT DESCARTES
  Goal:
    - Initialize a Descartes RobotModel object for carrying out various robot related tasks.
    - Initialize a Descartes Path Planner for planning a robot path from a trajectory.
    - Use the moveit MoveGroup interface to move the arm to a pre-recorded positions saved in the moveit config package.
    - Verify that the arm reached the target.
*/

namespace vsl_motion_planning
{

void VSLDescartesMoveitPlanner::initDescartes()
{
    // Instantiating a robot model
    robot_model_ptr_.reset(new descartes_moveit::IkFastMoveitStateAdapter);
    robot_model_ptr_->setCheckCollisions(true);

    if (robot_model_ptr_->initialize(ROBOT_DESCRIPTION_PARAM,
                                     config_.group_name,
                                     config_.world_frame,
                                     config_.tip_link))
    {
        ROS_INFO_STREAM("Descartes Robot Model initialized");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to initialize Robot Model");
        exit(-1);
    }

    bool succeeded = planner_.initialize(robot_model_ptr_);
    if (succeeded)
    {
        ROS_INFO_STREAM("Descartes Dense Planner initialized");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to initialize Dense Planner");
        exit(-1);
    }

    loadRobotModel();

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

void VSLDescartesMoveitPlanner::loadRobotModel()
{
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));

    kinematic_model_ = robot_model_loader_->getModel();
    if (!kinematic_model_)
    {
        ROS_ERROR_STREAM("Failed to load robot model from robot description parameter:robot_description");
        exit(-1);
    }
    joint_model_group_ = kinematic_model_->getJointModelGroup(config_.group_name);
    kinematic_state_.reset(new moveit::core::RobotState(kinematic_model_));
}

// void VSLDescartesMoveitPlanner::
// {
//     planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_));

//     /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
//                      the internal planning scene accordingly */
//     planning_scene_monitor_->startSceneMonitor();
//     /* listens to changes of world geometry, collision objects, and (optionally) octomaps
//                               world geometry, collision objects and optionally octomaps */
//     planning_scene_monitor_->startWorldGeometryMonitor();
//     /* listen to joint state updates as well as changes in attached collision objects
//                       and update the internal planning scene accordingly*/
//     planning_scene_monitor_->startStateMonitor();

//     /* We can get the most up to date robot state from the PlanningSceneMonitor by locking the internal planning scene
//    for reading. This lock ensures that the underlying scene isn't updated while we are reading it's state.
//    RobotState's are useful for computing the forward and inverse kinematics of the robot among many other uses */

//     kinematic_state_.reset(new robot_state::RobotState(planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState()));

//     planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(kinematic_model_, nh_, "planning_plugin", "request_adapters"));
//     /* First, set the state in the planning scene to the final state of the last plan */

//     kinematic_state_ = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentStateUpdated(response.trajectory_start);
//     kinematic_state_->setJointGroupPositions(joint_model_group_, response.trajectory.joint_trajectory.points.back().positions);
//     robot_state::robotStateToRobotStateMsg(*kinematic_state_, req.start_state);

//     robot_state::RobotState goal_state(*kinematic_state_);
//     std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0};
//     goal_state.setJointGroupPositions(joint_model_group, joint_values);
//     moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

//     req.goal_constraints.clear();
//     req.goal_constraints.push_back(joint_goal);

//     // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
//     // representation while planning
//     {
//         planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
//         /* Now, call the pipeline and check whether planning was successful. */
//         planning_pipeline->generatePlan(lscene, req, res);
//     }
//     if (res.error_code_.val != res.error_code_.SUCCESS)
//     {
//         ROS_ERROR("Could not compute plan successfully");
//         return 0;
//     }
//
// }

} // namespace vsl_motion_planning