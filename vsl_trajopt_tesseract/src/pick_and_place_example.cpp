#include <vsl_trajopt_planner.h>

// C++
#include <fstream>
// #include <jsoncpp/json/json.h>

// Tesseract
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
// #include <tesseract_environment/core/utils.h>

// Trajopt
#include <trajopt/file_write_callback.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";       /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot
                                                                          description */
const std::string GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes_rviz";
const std::string MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";

namespace vsl_motion_planner
{

ProblemConstructionInfo VSLTrajoptPlanner::cppMethod()
{
  ProblemConstructionInfo pci(tesseract_);

  tesseract_common::VectorIsometry3d tool_poses = makePuzzleToolPoses();

  // Populate Basic Info
  pci.basic_info.n_steps = static_cast<int>(tool_poses.size());
  pci.basic_info.manip = "manipulator";
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = false;

  pci.opt_info.max_iter = 200;
  pci.opt_info.min_approx_improve = 1e-3;
  pci.opt_info.min_trust_box_size = 1e-3;

  // Create Kinematic Object
  pci.kin = pci.getManipulator(pci.basic_info.manip);

  // Populate Init Info
  EnvState::ConstPtr current_state = pci.env->getCurrentState();
  Eigen::VectorXd start_pos;
  start_pos.resize(pci.kin->numJoints());
  int cnt = 0;
  for (const auto &j : pci.kin->getJointNames())
  {
    start_pos[cnt] = current_state->joints.at(j);
    ++cnt;
  }

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);
  //  pci.init_info.data.col(6) = VectorXd::LinSpaced(steps_, start_pos[6],
  //  end_pos[6]);

  // Populate Cost Info
  std::shared_ptr<JointVelTermInfo> joint_vel = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  joint_vel->coeffs = std::vector<double>(7, 1.0);
  joint_vel->targets = std::vector<double>(7, 0.0);
  joint_vel->first_step = 0;
  joint_vel->last_step = pci.basic_info.n_steps - 1;
  joint_vel->name = "joint_vel";
  joint_vel->term_type = TT_COST;
  pci.cost_infos.push_back(joint_vel);

  std::shared_ptr<JointAccTermInfo> joint_acc = std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  joint_acc->coeffs = std::vector<double>(7, 2.0);
  joint_acc->targets = std::vector<double>(7, 0.0);
  joint_acc->first_step = 0;
  joint_acc->last_step = pci.basic_info.n_steps - 1;
  joint_acc->name = "joint_acc";
  joint_acc->term_type = TT_COST;
  pci.cost_infos.push_back(joint_acc);

  std::shared_ptr<JointJerkTermInfo> joint_jerk = std::shared_ptr<JointJerkTermInfo>(new JointJerkTermInfo);
  joint_jerk->coeffs = std::vector<double>(7, 5.0);
  joint_jerk->targets = std::vector<double>(7, 0.0);
  joint_jerk->first_step = 0;
  joint_jerk->last_step = pci.basic_info.n_steps - 1;
  joint_jerk->name = "joint_jerk";
  joint_jerk->term_type = TT_COST;
  pci.cost_infos.push_back(joint_jerk);

  std::shared_ptr<CollisionTermInfo> collision = std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);
  collision->name = "collision";
  collision->term_type = TT_COST;
  collision->continuous = false;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 1;
  collision->gap = 1;
  collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 20);
  pci.cost_infos.push_back(collision);

  // Populate Constraints
  Eigen::Isometry3d grinder_frame = tesseract_->getEnvironment()->getLinkTransform("grinder_frame");
  Eigen::Quaterniond q(grinder_frame.linear());

  Eigen::Vector3d stationary_xyz = grinder_frame.translation();
  Eigen::Vector4d stationary_wxyz = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());

  for (auto i = 0; i < pci.basic_info.n_steps; ++i)
  {
    std::shared_ptr<CartPoseTermInfo> pose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
    pose->term_type = TT_CNT;
    pose->name = "waypoint_cart_" + std::to_string(i);
    pose->link = "part";
    pose->tcp = tool_poses[static_cast<unsigned long>(i)];
    pose->timestep = i;
    pose->xyz = stationary_xyz;
    pose->wxyz = stationary_wxyz;
    pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
    pose->rot_coeffs = Eigen::Vector3d(10, 10, 0);

    pci.cnt_infos.push_back(pose);
  }

  return pci;
}

bool VSLTrajoptPlanner::run()
{

  //////////////////
  /// ROS SETUP ///
  /////////////////

  // Pull ROS params
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  // Initialize the environment
  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create plotting tool
  tesseract_rosutils::ROSPlottingPtr plotter =
      std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract_->getEnvironment());

  if (rviz_)
  {
    // These are used to keep visualization updated
    modify_env_rviz_ = nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(MODIFY_ENVIRONMENT_SERVICE, false);
    get_env_changes_rviz_ =
        nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(GET_ENVIRONMENT_CHANGES_SERVICE, false);

    // Check RViz to make sure nothing has changed
    if (!checkRviz())
      return false;
  }

  ///////////////////
  /// ROBOT SETUP ///
  ///////////////////

  // Set the initial state of the robot
  std::unordered_map<std::string, double> initial_initial_joint_states;
  initial_joint_states["kr210_joint_a1"] = 0.0;
  initial_joint_states["kr210_joint_a2"] = -1.57;
  initial_joint_states["kr210_joint_a3"] = 1.57;
  initial_joint_states["kr210_joint_a4"] = 0.0;
  initial_joint_states["kr210_joint_a5"] = 0.0;
  initial_joint_states["kr210_joint_a6"] = 0.0;
  tesseract_->getEnvironment()->setState(initial_joint_states);

  if (rviz_)
  {
    // Now update rviz environment
    if (!sendRvizChanges(0))
      return false;
  }

  // Set Log Level
  util::gLogLevel = util::LevelInfo;








  ////////////
  /// PICK ///
  ////////////

  // Choose the manipulator and end effector link
  std::string manip = "manipulator";
  std::string end_effector = "penholder_tool_tip";

  // Define the final pose (on top of the box)
  Eigen::Isometry3d final_pose;
  Eigen::Quaterniond orientation(0.0, 0.0, 1.0, 0.0);
  final_pose.linear() = orientation.matrix();
  final_pose.translation() += Eigen::Vector3d(box_x, box_y, box_side + 0.77153); // Offset for the table

  // Define the approach pose
  Eigen::Isometry3d approach_pose = final_pose;
  approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.15);

  // Create the problem construction info
  trajopt::ProblemConstructionInfo pci(tesseract_);

  pci.basic_info.n_steps = steps_ * 2;
  pci.basic_info.manip = manip;
  pci.basic_info.dt_lower_lim = 2;   // 1/most time
  pci.basic_info.dt_upper_lim = 100; // 1/least time
  pci.basic_info.start_fixed = true;
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = pci.getManipulator(pci.basic_info.manip);

  pci.init_info.type = trajopt::InitInfo::STATIONARY;
  pci.init_info.dt = 0.5;

  // Add a collision cost
  if (true)
  {
    std::shared_ptr<trajopt::CollisionTermInfo> collision(new trajopt::CollisionTermInfo);
    collision->name = "collision";
    collision->term_type = trajopt::TT_COST;
    collision->continuous = true;
    collision->first_step = 1;
    collision->last_step = pci.basic_info.n_steps - 1;
    collision->gap = 1;
    collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 40);
    pci.cost_infos.push_back(collision);
  }

  // Add a velocity cost without time to penalize paths that are longer
  if (true)
  {
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);
    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 5.0);
    jv->term_type = trajopt::TT_COST;
    jv->first_step = 0;
    jv->last_step = pci.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cost";
    pci.cost_infos.push_back(jv);
  }

  // Add a velocity cnt with time to insure that robot dynamics are obeyed
  if (false)
  {
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);

    // Taken from iiwa documentation (radians/s) and scaled by 0.8
    std::vector<double> vel_lower_lim{1.71 * -0.8, 1.71 * -0.8, 1.75 * -0.8, 2.27 * -0.8,
                                      2.44 * -0.8, 3.14 * -0.8, 3.14 * -0.8};
    std::vector<double> vel_upper_lim{1.71 * 0.8, 1.71 * 0.8, 1.75 * 0.8, 2.27 * 0.8,
                                      2.44 * 0.8, 3.14 * 0.8, 3.14 * 0.8};

    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 50.0);
    jv->lower_tols = vel_lower_lim;
    jv->upper_tols = vel_upper_lim;
    jv->term_type = (trajopt::TT_CNT | trajopt::TT_USE_TIME);
    jv->first_step = 0;
    jv->last_step = pci.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cnt";
    pci.cnt_infos.push_back(jv);
  }

  // Add cartesian pose cnt at the approach point
  if (true)
  {
    Eigen::Quaterniond rotation(approach_pose.linear());
    std::shared_ptr<trajopt::CartPoseTermInfo> pose_constraint =
        std::shared_ptr<trajopt::CartPoseTermInfo>(new trajopt::CartPoseTermInfo);
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = end_effector;
    pose_constraint->timestep = steps_;
    pose_constraint->xyz = approach_pose.translation();

    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(steps_);
    pci.cnt_infos.push_back(pose_constraint);
  }

  // Add cartesian pose cnt at the final point
  if (true)
  {
    Eigen::Quaterniond rotation(final_pose.linear());
    std::shared_ptr<trajopt::CartPoseTermInfo> pose_constraint =
        std::shared_ptr<trajopt::CartPoseTermInfo>(new trajopt::CartPoseTermInfo);
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = end_effector;
    pose_constraint->timestep = 2 * steps_ - 1;
    pose_constraint->xyz = final_pose.translation();

    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(2 * steps_ - 1);
    pci.cnt_infos.push_back(pose_constraint);
  }

  // Add a cost on the total time to complete the pick
  if (false)
  {
    std::shared_ptr<trajopt::TotalTimeTermInfo> time_cost(new trajopt::TotalTimeTermInfo);
    time_cost->name = "time_cost";
    time_cost->coeff = 5.0;
    time_cost->limit = 0.0;
    time_cost->term_type = trajopt::TT_COST;
    pci.cost_infos.push_back(time_cost);
  }

  // Create the pick problem
  trajopt::TrajOptProb::Ptr pick_prob = ConstructProblem(pci);

  // Set the optimization parameters (Most are being left as defaults)
  tesseract_motion_planners::TrajOptPlannerConfig config(pick_prob);
  config.params.max_iter = 100;

  // Create Plot Callback
  if (plotting_)
  {
    config.callbacks.push_back(PlotCallback(*pick_prob, plotter));
  }

  // Create file write callback discarding any of the file's current contents
  std::shared_ptr<std::ofstream> stream_ptr(new std::ofstream);
  if (write_to_file_)
  {
    std::string path = ros::package::getPath("tesseract_ros_examples") + "/file_output_pick.csv";
    stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
    config.callbacks.push_back(trajopt::WriteCallback(stream_ptr, pick_prob));
  }

  // Create the planner and the responses that will store the results
  tesseract_motion_planners::TrajOptMotionPlanner planner;
  tesseract_motion_planners::PlannerResponse planning_response;
  tesseract_motion_planners::PlannerResponse planning_response_place;

  // Set Planner Configuration
  planner.setConfiguration(std::make_shared<tesseract_motion_planners::TrajOptPlannerConfig>(config));

  // Solve problem. Results are stored in the response
  planner.solve(planning_response);

  if (write_to_file_)
    stream_ptr->close();

  // Plot the resulting trajectory
  if (plotting_)
    plotter->plotTrajectory(pick_prob->GetKin()->getJointNames(),
                            planning_response.joint_trajectory.trajectory.leftCols(
                                static_cast<long>(pick_prob->GetKin()->getJointNames().size())));

  std::cout << planning_response.joint_trajectory.trajectory << '\n';

  /////////////
  /// PLACE ///
  /////////////

  if (rviz_)
  {
    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  // Detach the simulated box from the world and attach to the end effector
  Joint joint_box2("joint_box2");
  joint_box2.parent_link_name = end_effector;
  joint_box2.child_link_name = link_box.getName();
  joint_box2.type = JointType::FIXED;
  joint_box2.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  joint_box2.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(0, 0, box_side / 2.0);

  tesseract_->getEnvironment()->moveLink(joint_box2);
  tesseract_->getEnvironment()->addAllowedCollision(link_box.getName(), "iiwa_link_ee", "Never");
  tesseract_->getEnvironment()->addAllowedCollision(link_box.getName(), end_effector, "Adjacent");

  if (rviz_)
  {
    // Now update rviz environment
    if (!sendRvizChanges(1))
      return false;
  }

  // Set the current state to the last state of the pick trajectory
  tesseract_->getEnvironment()->setState(planning_response.joint_trajectory.joint_names,
                                         planning_response.joint_trajectory.trajectory.bottomRows(1).transpose());

  // Retreat to the approach pose
  Eigen::Isometry3d retreat_pose = approach_pose;

  // Define some place locations.
  Eigen::Isometry3d bottom_right_shelf, bottom_left_shelf, middle_right_shelf, middle_left_shelf, top_right_shelf,
      top_left_shelf;
  bottom_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  bottom_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 0.906);
  bottom_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  bottom_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 0.906);
  middle_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  middle_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 1.16);
  middle_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  middle_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 1.16);
  top_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  top_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 1.414);
  top_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  top_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 1.414);

  // Set the target pose to middle_left_shelf
  final_pose = middle_left_shelf;

  // Setup approach for place move
  approach_pose = final_pose;
  approach_pose.translation() += Eigen::Vector3d(0.0, -0.25, 0);

  // Create the problem construction info
  trajopt::ProblemConstructionInfo pci_place(tesseract_);

  pci_place.basic_info.n_steps = steps_ * 3;
  pci_place.basic_info.manip = manip;
  pci_place.basic_info.dt_lower_lim = 2;   // 1/most time
  pci_place.basic_info.dt_upper_lim = 100; // 1/least time
  pci_place.basic_info.start_fixed = true;
  pci_place.basic_info.use_time = false;

  // Create Kinematic Object
  pci_place.kin = pci_place.getManipulator(pci_place.basic_info.manip);

  pci_place.init_info.type = trajopt::InitInfo::STATIONARY;
  pci_place.init_info.dt = 0.5;

  // Add a collision cost
  if (true)
  {
    std::shared_ptr<trajopt::CollisionTermInfo> collision(new trajopt::CollisionTermInfo);
    collision->name = "collision";
    collision->term_type = trajopt::TT_COST;
    collision->continuous = true;
    collision->first_step = 1;
    collision->last_step = pci_place.basic_info.n_steps - 1;
    collision->gap = 1;
    collision->info = trajopt::createSafetyMarginDataVector(pci_place.basic_info.n_steps, 0.025, 40);
    pci_place.cost_infos.push_back(collision);
  }

  // Add a velocity cost without time to penalize paths that are longer
  if (true)
  {
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);
    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 5.0);
    jv->term_type = trajopt::TT_COST;
    jv->first_step = 0;
    jv->last_step = pci_place.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cost";
    pci_place.cost_infos.push_back(jv);
  }
  // Add a velocity cnt with time to insure that robot dynamics are obeyed
  if (false)
  {
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);

    // Taken from iiwa documentation (radians/s) and scaled by 0.8
    std::vector<double> vel_lower_lim{1.71 * -0.8, 1.71 * -0.8, 1.75 * -0.8, 2.27 * -0.8,
                                      2.44 * -0.8, 3.14 * -0.8, 3.14 * -0.8};
    std::vector<double> vel_upper_lim{1.71 * 0.8, 1.71 * 0.8, 1.75 * 0.8, 2.27 * 0.8,
                                      2.44 * 0.8, 3.14 * 0.8, 3.14 * 0.8};

    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 50.0);
    jv->lower_tols = vel_lower_lim;
    jv->upper_tols = vel_upper_lim;
    jv->term_type = (trajopt::TT_CNT | trajopt::TT_USE_TIME);
    jv->first_step = 0;
    jv->last_step = pci_place.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cnt";
    pci_place.cnt_infos.push_back(jv);
  }

  // Add cartesian pose cnt at the retreat point
  if (true)
  {
    Eigen::Quaterniond rotation(retreat_pose.linear());
    std::shared_ptr<trajopt::CartPoseTermInfo> pose_constraint =
        std::shared_ptr<trajopt::CartPoseTermInfo>(new trajopt::CartPoseTermInfo);
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = end_effector;
    pose_constraint->timestep = steps_ - 1;
    pose_constraint->xyz = retreat_pose.translation();

    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(steps_ - 1);
    pci_place.cnt_infos.push_back(pose_constraint);
  }

  // Add cartesian pose cnt at the final point
  int steps = 3 * steps_ - 2 * steps_;
  for (int index = 0; index < steps; index++)
  {
    Eigen::Quaterniond rotation(final_pose.linear());
    std::shared_ptr<trajopt::CartPoseTermInfo> pose_constraint =
        std::shared_ptr<trajopt::CartPoseTermInfo>(new trajopt::CartPoseTermInfo);
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = end_effector;
    pose_constraint->timestep = 2 * steps_ + index;
    pose_constraint->xyz = approach_pose.translation();
    pose_constraint->xyz.y() = approach_pose.translation().y() + 0.25 / (steps - 1) * index;

    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(2 * steps_ + index);
    pci_place.cnt_infos.push_back(pose_constraint);
  }

  // Add a cost on the total time to complete the pick
  if (false)
  {
    std::shared_ptr<trajopt::TotalTimeTermInfo> time_cost(new trajopt::TotalTimeTermInfo);
    time_cost->name = "time_cost";
    time_cost->coeff = 5.0;
    time_cost->term_type = trajopt::TT_COST;
    pci_place.cost_infos.push_back(time_cost);
  }

  // Create the place problem
  trajopt::TrajOptProb::Ptr place_prob = ConstructProblem(pci_place);

  // Set the optimization parameters
  tesseract_motion_planners::TrajOptPlannerConfig config_place(place_prob);
  config_place.params.max_iter = 100;

  // Create Plot Callback
  if (plotting_)
  {
    config_place.callbacks.push_back(PlotCallback(*place_prob, plotter));
  }

  // Create file write callback discarding any of the file's current contents
  std::shared_ptr<std::ofstream> stream_ptr_place(new std::ofstream);
  if (write_to_file_)
  {
    std::string path = ros::package::getPath("pick_and_place") + "/file_output_place.csv";
    stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
    config_place.callbacks.push_back(trajopt::WriteCallback(stream_ptr_place, place_prob));
  }

  // Set Planner Configuration
  planner.setConfiguration(std::make_shared<tesseract_motion_planners::TrajOptPlannerConfig>(config_place));

  // Solve problem
  planner.solve(planning_response_place);

  if (write_to_file_)
    stream_ptr_place->close();

  // Plot the resulting trajectory
  if (plotting_)
    plotter->plotTrajectory(planning_response_place.joint_trajectory.joint_names,
                            planning_response_place.joint_trajectory.trajectory.leftCols(
                                static_cast<long>(place_prob->GetKin()->getJointNames().size())));

  std::cout << planning_response_place.joint_trajectory.trajectory << '\n';

  ROS_INFO("Done");
  return true;
}
} // namespace vsl_motion_planner
