// #include <tesseract_motion_planners/descartes/descartes_motion_planner.h>

// #include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>
// #include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

// std::vector<std::vector<descartes_light::PositionSamplerPtr>>
// makeSamplers(const ToolPath& path, descartes_light::CollisionInterfacePtr coll_env)
// {
//   // The current setup requires that our cartesian sampler is aware of the robot
//   // kinematics
//   opw_kinematics::Parameters<double> kin_params = makeIrb4600_205_60<double>();
//   const auto tip_to_tool = hybrid_planning_common::torchTool0ToTCP();
//   const auto world_to_base = Eigen::Isometry3d::Identity();

//   descartes_light::KinematicsInterface kin_interface (kin_params, world_to_base, tip_to_tool);

//   std::vector<std::vector<descartes_light::PositionSamplerPtr>> result (path.size());

//   for (std::size_t i = 0; i < path.size(); ++i)
//   {
//     const auto& pass = path[i];
//     for (const auto& pose : pass)
//     {
//       auto collision_clone = descartes_light::CollisionInterfacePtr(coll_env->clone());
//       auto sampler = std::make_shared<descartes_light::SpoolSampler>(pose, kin_interface, collision_clone);
//       result[i].push_back(std::move(sampler));
//     }
//   }

//   return result;
// }

//   hybrid_planning_common::SamplerConfiguration sampler_config;
//   auto collision_iface =
//       std::make_shared<descartes_light::TesseractCollision>(env_def.environment, env_def.group_name);
//   sampler_config.samplers = makeSamplers(path_def.path, collision_iface);


// Opw
#include <opw_kinematics/opw_parameters.h>

// Descartes light
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>

// Tesseract
#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_collision_edge_evaluator.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/utils.h>
#include <tesseract_motion_planners/hybrid/descartes_trajopt_array_planner.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_motion_planners;

namespace vsl_motion_planner
{

DescartesMotionPlannerConfig createDescartesPlannerConfig(const Tesseract::ConstPtr& tesseract_ptr,
                             const std::string& /*manip*/,
                             const InverseKinematics::ConstPtr& kin,
                             const Eigen::Isometry3d& tcp,
                             const double robot_reach,
                             const EnvState::ConstPtr& current_state,
                             const std::vector<Waypoint::Ptr>& waypoints,
                             bool use_collision_edge_evaluator = false)
{
  const std::vector<std::string>& joint_names = kin->getJointNames();
  const std::vector<std::string>& active_link_names = kin->getActiveLinkNames();

  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      tesseract_ptr->getEnvironmentConst()->getSceneGraph(), active_link_names, current_state->transforms);

  // Create Collision Interface
  typename descartes_light::CollisionInterface<double>::Ptr coll_interface =
      std::make_shared<DescartesCollision<double>>(
          tesseract_ptr->getEnvironmentConst(), adjacency_map->getActiveLinkNames(), joint_names);

  // Create Timing Constraint
  std::vector<descartes_core::TimingConstraint<double>> timing = makeTiming<double>(waypoints, std::numeric_limits<double>::max());

  // Create Edge Evaluator
  descartes_light::EdgeEvaluator<double>::Ptr edge_computer;
  if (!use_collision_edge_evaluator)
  {
    edge_computer = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>(kin->numJoints());
  }
  else
  {
    edge_computer = std::make_shared<DescartesCollisionEdgeEvaluator<double>>(
        tesseract_ptr->getEnvironmentConst(), adjacency_map->getActiveLinkNames(), joint_names);
  }

  // Create is valid function
  DescartesIsValidFn<double> is_valid_fn =
      std::bind(&tesseract_kinematics::isWithinLimits<double>, std::placeholders::_1, kin->getLimits());

  // Create Position Samplers
  std::vector<typename descartes_light::PositionSampler<double>::Ptr> position_samplers =
      makeRobotSamplers<double>(waypoints, kin, coll_interface, current_state, tcp, robot_reach, true, is_valid_fn);

  return DescartesMotionPlannerConfigD(tesseract_ptr, adjacency_map->getActiveLinkNames(), joint_names, edge_computer, timing, position_samplers, waypoints);
}




// tesseract::tesseract_planning::DescartesPlanner descartes_planner;

tesseract_common::VectorIsometry3d tool_poses = getCourse();

DescartesMotionPlannerConfig descartes_config(tesseract_, tesseract_->getEnvironment()->getActiveLinkNames(), tesseract_->getEnvironment()->getJointNames(), edge_evaluator, timing_contraint, samplers, waypoints );

  DescartesMotionPlannerConfig(tesseract::Tesseract::ConstPtr tesseract_ptr,
                               const std::vector<std::string> active_link_names,
                               const std::vector<std::string> joint_names,
                               const typename descartes_light::EdgeEvaluator<FloatType>::Ptr& edge_evaluator,
                               std::vector<descartes_core::TimingConstraint<FloatType>> timing_constraint,
                               std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> samplers,
                               const std::vector<Waypoint::Ptr>& waypoints)





// tesseract::tesseract_planning::TrajOptPlanner trajopt_planner;

ProblemConstructionInfo pci = cppMethod();
TrajOptProb::Ptr prob = ConstructProblem(pci);
tesseract::tesseract_planning::TrajOptPlannerConfig trajopt_config(prob);



tesseract::tesseract_planning::DescartesTrajOptArrayPlanner descartes_trajopt_planner;


descartes_trajopt_planner.setConfiguration(descartes_config,trajopt_config);

tesseract::tesseract_planning::PlannerResponse planning_response;

descartes_trajopt_planner.solve(planning_response, true);


Eigen::Quaterniond orientation(0.0, 0.0, 1.0, 0.0);

std::string manip = "Manipulator";
std::string end_effector = "iiwa_link_ee";



TrajoptPickAndPlaceConstructor prob_constructor(tesseract_->getEnvironment(), manip, end_effector);

// Define the final pose
Eigen::Isometry3d final_pose;
final_pose.linear() = orientation.matrix();
final_pose.translation() = world_to_box.translation();
double gripper_offset = 0.08;
final_pose.translation() += Eigen::Vector3d(0.0, 0.0, gripper_offset); // We add an offset for a gripper since it's not in the URDF

// Define the approach pose
Eigen::Isometry3d approach_pose = final_pose;
approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.15);

// Create and solve pick problem
trajopt::TrajOptProbPtr pick_prob =
    prob_constructor.generatePickProblem(approach_pose, final_pose, steps_per_phase);

// Set the optimization parameters (Most are being left as defaults)
tesseract::tesseract_planning::TrajOptPlannerConfig config(pick_prob);
config.params.max_iter = 500;

// Create Plot Callback
if (plotting_cb)
{
    tesseract::tesseract_ros::ROSBasicPlottingPtr plotter_ptr(new tesseract::tesseract_ros::ROSBasicPlotting(tesseract_->getEnvironment()));
    config.callbacks.push_back(PlotCallback(*pick_prob, plotter_ptr));
}
// Create file write callback discarding any of the file's current contents
std::shared_ptr<std::ofstream> stream_ptr(new std::ofstream);
if (file_write_cb)
{
    std::string path = ros::package::getPath("pick_and_place") + "/file_output_pick.csv";
    stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
    config.callbacks.push_back(trajopt::WriteCallback(stream_ptr, pick_prob));
}

// Solve problem
planner.solve(planning_response, config);

plotter.plotTrajectory(env->getJointNames(), planning_response.trajectory.leftCols(tesseract_->getEnvironment()->getJointNames().size()));
std::cout << planning_response.trajectory << '\n';

traj_msg3 = trajArrayToJointTrajectoryMsg(planning_response.joint_names, planning_response.trajectory, false, t1);

//         // Set the current state to the last state of the trajectory
// env->setState(tesseract_->getEnvironment()->getJointNames(), planning_response.trajectory.bottomRows(1).transpose());