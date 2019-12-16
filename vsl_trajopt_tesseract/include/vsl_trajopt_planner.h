#ifndef VSL_TRAJOPT_PLANNER_H
#define VSL_TRAJOPT_PLANNER_H

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/service_client.h>

// C++
#include <fstream>
#include <string>
#include <memory>
// #include <jsoncpp/json/json.h>

// Tesseract
#include <tesseract/tesseract.h>
#include <tesseract_common/macros.h>
#include <tesseract_common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <tesseract_rosutils/conversions.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
// #include <tesseract_environment/core/utils.h>

// Trajopt
#include <trajopt/file_write_callback.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>


namespace vsl_motion_planner
{
/**
 * @brief An example of a robot picking up a box and placing it on a shelf leveraging
 * tesseract and trajopt to generate the motion trajectory.
 */
class VSLTrajoptPlanner : public Example
{
public:
  VSLTrajoptPlanner(ros::NodeHandle nh, bool plotting, bool rviz, int steps, bool write_to_file)
    : Example(plotting, rviz), nh_(nh), steps_(steps), write_to_file_(write_to_file)
  {
  }
  ~VSLTrajoptPlanner() = default;

  bool run() override;

private:
  ros::NodeHandle nh_;
  int steps_;
  bool write_to_file_;
};

}  // namespace vsl_trajopt_tesseract

#endif
