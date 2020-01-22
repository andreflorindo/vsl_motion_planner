#include <vsl_descartes_moveit_planner.h>

/* GENERATE TRAJECTORY
  Goal:
    - Create a Descartes Trajectory from an array of poses.
    - Create trajectory points that are free to rotate about the tool's z axis
*/

namespace vsl_motion_planning
{

void VSLDescartesMoveitPlanner::generateTrajectory(EigenSTL::vector_Isometry3d &poses, std::vector<descartes_core::TrajectoryPtPtr> &input_traj)
{
    using namespace descartes_core;
    using namespace descartes_trajectory;

    // creating descartes trajectory points
    input_traj.clear();
    input_traj.reserve(poses.size());
    for (unsigned int i = 0; i < poses.size(); i++)
    {
        const Eigen::Isometry3d &single_pose = poses[i];

        //Create AxialSymetricPt objects in order to define a trajectory cartesian point with rotational freedom about the tool's z axis.
        descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
            new descartes_trajectory::AxialSymmetricPt(single_pose, ORIENTATION_INCREMENT, descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS));

        // saving points into trajectory
        input_traj.push_back(pt);
    }

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

} // namespace vsl_motion_planning
