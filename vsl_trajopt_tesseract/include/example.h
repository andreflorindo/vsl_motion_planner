




namespace vsl_motion_planner
{
/**
 * @brief The Example base class
 *
 * It provides a generic interface for all examples as a library which then
 * can easily be integrated as unit tests so breaking changes are caught.
 * Also it provides a few utility functions for checking rviz environment and
 * updating the rviz environment.
 */
class Example
{
public:
  Example(bool plotting, bool rviz)
    : plotting_(plotting), rviz_(rviz), tesseract_(std::make_shared<tesseract::Tesseract>())
  {
  }
  virtual ~Example() = default;

  virtual bool run() = 0;

protected:
  bool plotting_;                           /**< @brief Enable plotting so data is published for rviz if available */
  bool rviz_;                               /**< @brief Enable rviz updating */
  tesseract::Tesseract::Ptr tesseract_;     /**< @brief Tesseract Manager Class */
  ros::ServiceClient modify_env_rviz_;      /**< @brief Service for modifying tesseract environment in rviz */
  ros::ServiceClient get_env_changes_rviz_; /**< @brief Get the environment changes from rviz */

  /**
   * @brief Check rviz and make sure the rviz environment revision number is zero.
   * @return True if revision number is zero, otherwise false.
   */
  bool checkRviz()
  {
    // Get the current state of the environment.
    // Usually you would not be getting environment state from rviz
    // this is just an example. You would be gettting it from the
    // environment_monitor node. Need to update examples to launch
    // environment_monitor node.
    get_env_changes_rviz_.waitForExistence();
    tesseract_msgs::GetEnvironmentChanges env_changes;
    env_changes.request.revision = 0;
    if (get_env_changes_rviz_.call(env_changes))
    {
      ROS_INFO("Retrieve current environment changes!");
    }
    else
    {
      ROS_ERROR("Failed to retrieve current environment changes!");
      return false;
    }

    // There should not be any changes but check
    if (env_changes.response.revision != 0)
    {
      ROS_ERROR("The environment has changed externally!");
      return false;
    }
    return true;
  }

  /**
   * @brief Send RViz the latest number of commands
   * @param n The past revision number
   * @return True if successful otherwise false
   */
  bool sendRvizChanges(unsigned long past_revision)
  {
    modify_env_rviz_.waitForExistence();
    tesseract_msgs::ModifyEnvironment update_env;
    update_env.request.id = tesseract_->getEnvironment()->getName();
    update_env.request.revision = past_revision;
    if (!tesseract_rosutils::toMsg(update_env.request.commands,
                                   tesseract_->getEnvironment()->getCommandHistory(),
                                   update_env.request.revision))
    {
      ROS_ERROR("Failed to generate commands to update rviz environment!");
      return false;
    }

    if (modify_env_rviz_.call(update_env))
    {
      ROS_INFO("RViz environment Updated!");
    }
    else
    {
      ROS_INFO("Failed to update rviz environment");
      return false;
    }

    return true;
  }
};

}  // namespace vsl_trajopt_tesseract

