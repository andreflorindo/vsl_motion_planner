#include <vsl_trajopt_planner.h>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;

namespace vsl_motion_planner
{

// tesseract_common::VectorIsometry3d PuzzlePieceExample::makePuzzleToolPoses()
// {
//   tesseract_common::VectorIsometry3d path; // results
//   std::ifstream indata;                    // input file

//   // You could load your parts from anywhere, but we are transporting them with
//   // the git repo
//   std::string filename = ros::package::getPath("tesseract_ros_examples") + "/config/puzzle_bent.csv";

//   // In a non-trivial app, you'll of course want to check that calls like 'open'
//   // succeeded
//   indata.open(filename);

//   std::string line;
//   int lnum = 0;
//   while (std::getline(indata, line))
//   {
//     ++lnum;
//     if (lnum < 3)
//       continue;

//     std::stringstream lineStream(line);
//     std::string cell;
//     Eigen::Matrix<double, 6, 1> xyzijk;
//     int i = -2;
//     while (std::getline(lineStream, cell, ','))
//     {
//       ++i;
//       if (i == -1)
//         continue;

//       xyzijk(i) = std::stod(cell);
//     }

//     Eigen::Vector3d pos = xyzijk.head<3>();
//     pos = pos / 1000.0; // Most things in ROS use meters as the unit of length.
//                         // Our part was exported in mm.
//     Eigen::Vector3d norm = xyzijk.tail<3>();
//     norm.normalize();

//     // This code computes two extra directions to turn the normal direction into
//     // a full defined frame. Descartes
//     // will search around this frame for extra poses, so the exact values do not
//     // matter as long they are valid.
//     Eigen::Vector3d temp_x = (-1 * pos).normalized();
//     Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
//     Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();
//     Eigen::Isometry3d pose;
//     pose.matrix().col(0).head<3>() = x_axis;
//     pose.matrix().col(1).head<3>() = y_axis;
//     pose.matrix().col(2).head<3>() = norm;
//     pose.matrix().col(3).head<3>() = pos;

//     path.push_back(pose);
//   }
//   indata.close();

//   return path;
// }

VSLTrajoptPlanner::VSLTrajoptPlanner() {}
VSLTrajoptPlanner::~VSLTrajoptPlanner() {}

void VSLTrajoptPlanner::initRos()
{
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");

  if (ph.getParam("group_name", config_.group_name) &&
      ph.getParam("tip_link", config_.tip_link) &&
      ph.getParam("base_link", config_.base_link) &&
      ph.getParam("world_frame", config_.world_frame) &&
      ph.getParam("plotting", config_.plotting) &&
      ph.getParam("rviz", config_.rviz))
      // nh.getParam("controller_joint_names", config_.joint_names))
  {
    ROS_INFO_STREAM("Loaded application parameters");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load application parameters");
    exit(-1);
  }

  ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

tesseract_common::VectorIsometry3d VSLTrajoptPlanner::getCourse()
{
  // Initialize Service client
  if (ros::service::waitForService(POSE_BUILDER_SERVICE, ros::Duration(SERVER_TIMEOUT)))
  {
    ROS_INFO_STREAM("Connected to '" << POSE_BUILDER_SERVICE << "' service");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to connect to '" << POSE_BUILDER_SERVICE << "' service");
    exit(-1);
  }

  pose_builder_client_ = nh_.serviceClient<vsl_msgs::PoseBuilder>(POSE_BUILDER_SERVICE);
  vsl_msgs::PoseBuilder srv;
  // srv.request.num_layer = num_layer;
  // srv.request.num_course = num_course;
  // ROS_INFO_STREAM("Requesting pose in base frame: " << num_layer);

  if (!pose_builder_client_.call(srv))
  {
    ROS_ERROR_STREAM("Failed to call '" << POSE_BUILDER_SERVICE << "' service");
    exit(-1);
  }

  tesseract_common::VectorIsometry3d poses;
  Eigen::Isometry3d single_pose;
  poses.reserve(srv.response.single_course_poses.poses.size());

  // Modify the single_pose type from PoseArray to Isometry3d
  for (unsigned int i = 0; i < srv.response.single_course_poses.poses.size(); i++)
  {
    tf::poseMsgToEigen(srv.response.single_course_poses.poses[i], single_pose);
    poses.emplace_back(single_pose);
  }
  return poses;
}

ProblemConstructionInfo VSLTrajoptPlanner::cppMethod()
{
  ProblemConstructionInfo pci(tesseract_);
  tesseract_common::VectorIsometry3d tool_poses = getCourse();

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

  // Get Init Pose
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
  joint_vel->coeffs = std::vector<double>(6, 1.0); // 7 double with value 1
  joint_vel->targets = std::vector<double>(6, 0.0);
  // joint_vel->upper_tols = std::vector<double>(6, 0.0);
  // joint_vel->lower_tols = std::vector<double>(6, 0.0);
  joint_vel->first_step = 0;
  joint_vel->last_step = pci.basic_info.n_steps - 1;
  joint_vel->name = "joint_vel";
  joint_vel->term_type = TT_COST;
  pci.cost_infos.push_back(joint_vel);

  std::shared_ptr<JointAccTermInfo> joint_acc = std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  joint_acc->coeffs = std::vector<double>(7, 2.0);
  joint_acc->targets = std::vector<double>(7, 0.0);
  // joint_acc->upper_tols = std::vector<double>(6, 0.0);
  // joint_acc->lower_tols = std::vector<double>(6, 0.0);
  joint_acc->first_step = 0;
  joint_acc->last_step = pci.basic_info.n_steps - 1;
  joint_acc->name = "joint_acc";
  joint_acc->term_type = TT_COST;
  pci.cost_infos.push_back(joint_acc);

  std::shared_ptr<JointJerkTermInfo> joint_jerk = std::shared_ptr<JointJerkTermInfo>(new JointJerkTermInfo);
  joint_jerk->coeffs = std::vector<double>(7, 5.0);
  joint_jerk->targets = std::vector<double>(7, 0.0);
  // joint_vel->upper_tols = std::vector<double>(6, 0.0);
  // joint_vel->lower_tols = std::vector<double>(6, 0.0);
  joint_jerk->first_step = 0;
  joint_jerk->last_step = pci.basic_info.n_steps - 1;
  joint_jerk->name = "joint_jerk";
  joint_jerk->term_type = TT_COST;
  pci.cost_infos.push_back(joint_jerk);

  std::shared_ptr<CollisionTermInfo> collision = std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);
  collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 20);
  collision->name = "collision";
  collision->term_type = TT_COST;
  collision->continuous = false;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 1;
  collision->gap = 1;
  pci.cost_infos.push_back(collision);

  // Populate Constraints

  std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);
  std::vector<double> vel_lower_lim{-2.14, -2.00, -1.95, -3.12, -3.00, -3.82};
  std::vector<double> vel_upper_lim{2.14, 2.00, 1.95, 3.12, 3.00, 3.82};
  jv->coeffs = std::vector<double>(7, 50.0);
  jv->targets = std::vector<double>(7, 0.0);
  jv->lower_tols = vel_lower_lim;
  jv->upper_tols = vel_upper_lim;
  jv->term_type = (trajopt::TT_CNT | trajopt::TT_USE_TIME);
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->name = "joint_velocity_cnt";
  pci.cnt_infos.push_back(jv);

  for (auto i = 0; i < pci.basic_info.n_steps; ++i)
  {
    Eigen::Isometry3d current_pose = tool_poses[static_cast<unsigned long>(i)];
    Eigen::Quaterniond q(current_pose.linear());
    Eigen::Vector3d current_xyz = current_pose.translation();
    Eigen::Vector4d current_wxyz = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());

    std::shared_ptr<CartPoseTermInfo> pose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
    pose->term_type = TT_CNT;
    pose->name = "waypoint_cart_" + std::to_string(i);
    pose->link = config_.tip_link;
    // pose->tcp = current_pose;
    pose->timestep = i;
    pose->xyz = current_xyz;
    pose->wxyz = current_wxyz;
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
  initRos();

  // Initialize the environment
  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_->init(ROBOT_DESCRIPTION_PARAM, ROBOT_SEMANTIC_PARAM, locator))
    return false;

  ROS_INFO_STREAM("Good");

  // Create plotting tool
  tesseract_rosutils::ROSPlottingPtr plotter =
      std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract_->getEnvironment());

  ROS_INFO_STREAM("Moring");

  if (config_.rviz)
  {
    // These are used to keep visualization updated
    modify_env_rviz_ = nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(MODIFY_ENVIRONMENT_SERVICE, false);
    get_env_changes_rviz_ =
        nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(GET_ENVIRONMENT_CHANGES_SERVICE, false);

    // Check RViz to make sure nothing has changed
    if (!checkRviz())
      return false;
  }

  // ///////////////////
  // /// ROBOT SETUP ///
  // ///////////////////

  // // Set the initial state of the robot
  // std::unordered_map<std::string, double> initial_joint_states;
  // initial_joint_states["kr210_joint_a1"] = 0.0;
  // initial_joint_states["kr210_joint_a2"] = -1.57;
  // initial_joint_states["kr210_joint_a3"] = 1.57;
  // initial_joint_states["kr210_joint_a4"] = 0.0;
  // initial_joint_states["kr210_joint_a5"] = 0.0;
  // initial_joint_states["kr210_joint_a6"] = 0.0;
  // tesseract_->getEnvironment()->setState(initial_joint_states);

  // if (config_.rviz)
  // {
  //   // Now update rviz environment
  //   if (!sendRvizChanges(0))
  //     return false;
  // }

  // // Set Log Level
  // util::gLogLevel = util::LevelInfo;

  // ////////////////////
  // /// SETUP PROBLEM //
  // ////////////////////

  // // Setup Problem
  // ProblemConstructionInfo pci = cppMethod();
  // TrajOptProb::Ptr prob = ConstructProblem(pci);

  // // Solve Trajectory
  // ROS_INFO("Trajectory is planned");

  // std::vector<ContactResultMap> collisions;
  // ContinuousContactManager::Ptr manager = prob->GetEnv()->getContinuousContactManager();
  // AdjacencyMap::Ptr adjacency_map =
  //     std::make_shared<tesseract_environment::AdjacencyMap>(prob->GetEnv()->getSceneGraph(),
  //                                                           prob->GetKin()->getActiveLinkNames(),
  //                                                           prob->GetEnv()->getCurrentState()->transforms);

  // manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  // manager->setContactDistanceThreshold(0);
  // collisions.clear();
  // bool found =
  //     checkTrajectory(*manager, *prob->GetEnv(), prob->GetKin()->getJointNames(), prob->GetInitTraj(), collisions);

  // ROS_INFO((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  // sco::BasicTrustRegionSQP opt(prob);
  // opt.setParameters(pci.opt_info);
  // if (config_.plotting)
  // {
  //   opt.addCallback(PlotCallback(*prob, plotter));
  // }

  // opt.initialize(trajToDblVec(prob->GetInitTraj()));
  // ros::Time tStart = ros::Time::now();
  // sco::OptStatus status = opt.optimize();
  // ROS_INFO("Optimization Status: %s, Planning time: %.3f",
  //          sco::statusToString(status).c_str(),
  //          (ros::Time::now() - tStart).toSec());

  // if (config_.plotting)
  //   plotter->clear();

  // collisions.clear();
  // found = checkTrajectory(
  //     *manager, *prob->GetEnv(), prob->GetKin()->getJointNames(), getTraj(opt.x(), prob->GetVars()), collisions);

  // ROS_INFO((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));

  // plotter->plotTrajectory(prob->GetKin()->getJointNames(), getTraj(opt.x(), prob->GetVars()));

  return true;
}

} // namespace vsl_motion_planner
