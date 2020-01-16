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