/***********************************************************
 *
 * @file: path_visual_plugin.h
 * @breif: Contains path visualization Rviz plugin class
 * @author: Yang Haodong
 * @update: 2023-10-2
 * @version: 1.1
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PATHVISUALPLUGIN_H
#define PATHVISUALPLUGIN_H

#include <QWidget>
#include <rviz/panel.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

QT_BEGIN_NAMESPACE
namespace Ui
{
class PathVisualPlugin;
}
QT_END_NAMESPACE

namespace path_visual_plugin
{
class PathVisualPlugin : public rviz::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new Path Visualization Plugin object
   */
  PathVisualPlugin(QWidget* parent = nullptr);

  /**
   * @brief Destroy the Path Visualization Plugin object
   */
  ~PathVisualPlugin();

  /**
   * @brief Publish planning path
   * @param path  planning path
   */
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
  /**
   * @brief User interface parameters initialization
   */
  void setupUi();
  /**
   * @brief ROS parameters initialization
   */
  void setupROS();

protected Q_SLOTS:
  /**
   *  @brief update the goal point, it is a callback funciton
   *  @param  pose    the goal setting in Rviz
   */
  void _onGoalUpdate(const geometry_msgs::PoseStamped::ConstPtr& pose);
  /**
   *  @brief update the start point, it is a callback funciton
   *  @param  pose    the start setting in Rviz
   */
  void _onStartUpdate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
  /**
   *  @brief call path planning service, it is a callback funciton
   */
  void _onPlanPath();

private:
  Ui::PathVisualPlugin* ui;  // ui object

  ros::Publisher marker_pub_;  // map marker publisher
  ros::Publisher plan_pub_;    // path planning publisher

  // start and goal point subscriber
  ros::Subscriber start_sub_, goal_sub_;

  // call plan client
  ros::ServiceClient call_plan_client_;

  //   start and goal point
  double start_x_, start_y_;
  double goal_x_, goal_y_;

  std::vector<std::string> planner_list_;  // valid planner name list
};

}  // namespace path_visual_plugin
#endif  // PATHVISUALPLUGIN_H
