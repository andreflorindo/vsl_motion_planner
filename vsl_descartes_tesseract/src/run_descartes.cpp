#include <vsl_descartes_tesseract_planner.h>

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

bool VSLDescartesTesseractPlanner::run()
{
    //////////////////
    /// ROS SETUP ///
    /////////////////

    // Pull ROS params
    initRos();

    // Initialize the environment
    std::string urdf_xml_string, srdf_xml_string;
    nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
    nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

    ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    {
        ROS_ERROR_STREAM("Tesseract failed to connect to the robot files");
        return false;
    }
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

    opw_params_.a1 = (0.350);
    opw_params_.a2 = (0.041);
    opw_params_.b = (0.000);
    opw_params_.c1 = (0.675);
    opw_params_.c2 = (1.150);
    opw_params_.c3 = (1.200);
    opw_params_.c4 = (0.215);

    opw_params_.offsets[2] = -M_PI / 2.0;

    auto robot_kin = tesseract_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
    auto opw_kin = std::make_shared<OPWInvKin>();
    opw_kin->init("manipulator",
                  opw_params_,
                  robot_kin->getBaseLinkName(),
                  robot_kin->getTipLinkName(),
                  robot_kin->getJointNames(),
                  robot_kin->getLinkNames(),
                  robot_kin->getActiveLinkNames(),
                  robot_kin->getLimits());

    tesseract_->getInvKinematicsManager()->addInvKinematicSolver(opw_kin);

    // Set the initial state of the robot
    std::unordered_map<std::string, double> initial_joint_states;
    initial_joint_states["kr210_joint_a1"] = 0.3;
    initial_joint_states["kr210_joint_a2"] = -0.76;
    initial_joint_states["kr210_joint_a3"] = 1.72;
    initial_joint_states["kr210_joint_a4"] = 0.0;
    initial_joint_states["kr210_joint_a5"] = 0.6;
    initial_joint_states["kr210_joint_a6"] = 0.0;
    tesseract_->getEnvironment()->setState(initial_joint_states);

    if (rviz_)
    {
        // Now update rviz environment
        if (!sendRvizChanges(0))
            return false;
    }

    /////////////////////////////////////
    /// DESCARTES PROBLEM CONSTRUCTION //
    /////////////////////////////////////

    // These specify the series of points to be optimized
    std::vector<Waypoint::Ptr> waypoints = getCourse();

    // Set Log Level
    //util::gLogLevel = util::LevelInfo;

    auto robot_kin_ik = tesseract_->getInvKinematicsManagerConst()->getInvKinematicSolver("manipulator", "OPWInvKin");
    auto current_state = tesseract_->getEnvironmentConst()->getCurrentState();

    DescartesMotionPlannerConfigD config = createDescartesPlannerConfig(
        tesseract_, "manipulator", robot_kin_ik, Eigen::Isometry3d::Identity(), 1.5, current_state, waypoints, true);

    // config.num_threads = 1;

    DescartesMotionPlanner<double> single_descartes_planner;
    PlannerResponse single_planner_response;
    single_descartes_planner.setConfiguration(config);
    auto single_status = single_descartes_planner.solve(single_planner_response, true);

    // Solve Trajectory
    ROS_INFO("Trajectory is planned");

    // tesseract_planning::DescartesTrajOptArrayPlanner descartes_trajopt_planner;

    // descartes_trajopt_planner.setConfiguration(descartes_config, trajopt_config);

    // // ///////////////////////
    // // /// FINAL TRAJECTORY //
    // // ///////////////////////

    // // getTraj() - To get trajectory of type trajopt::TrajArray

    if (plotting_)
        plotter->clear();

    // plotter->plotTrajectory(robot_kin->getJointNames(), single_planner_response.trajectory.leftCols(robot_kin->getJointNames().size()));

    // //////////////
    // /// EXECUTE //
    // //////////////

    // trajectory_msgs::JointTrajectory traj_msg;
    // ros::Duration t1(0.10);
    // traj_msg = trajArrayToJointTrajectoryMsg(prob->GetKin()->getJointNames(), getTraj(opt.x(), prob->GetVars()), pci.basic_info.use_time, t1);
    // joint_traj_.publish(traj_msg);

    // // Create action message
    // control_msgs::FollowJointTrajectoryGoal goal;
    // goal.trajectory = traj_msg;

    // ROS_INFO_STREAM("Robot path sent for execution");

    // if (follow_joint_trajectory_client_->sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     ROS_INFO_STREAM("Robot path execution completed");
    // }
    // else
    // {
    //     ROS_ERROR_STREAM("Failed to run robot path with error " << *follow_joint_trajectory_client_->getResult());
    //     exit(-1);
    // }

    // ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");

    return true;
}
} // namespace vsl_motion_planner