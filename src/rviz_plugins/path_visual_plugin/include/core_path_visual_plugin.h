/***********************************************************
 *
 * @file: core_path_visual_plugin.h
 * @breif: Contains core of path visualization Rviz plugin class
 * @author: Yang Haodong, Wu Maojia
 * @update: 2023-10-27
 * @version: 2.0
 *
 * Copyright (c) 2023， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef CORE_PATH_VISUAL_PLUGIN_H
#define CORE_PATH_VISUAL_PLUGIN_H

#include <rviz/panel.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "include/path_list.h"

namespace path_visual_plugin
{
class CorePathVisualPlugin : public QObject
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new CorePathVisualPlugin object
   */
  CorePathVisualPlugin();

  /**
   * @brief Destroy the CorePathVisualPlugin object
   */
  ~CorePathVisualPlugin();

  /**
   * @brief ROS parameters initialization
   */
  void setupROS();

  /**
   *  @brief call path planning service
   */
  void addPath(const QString& planner_name);

  /**
   *  @brief load paths file
   *  @param open_file  load paths from local workspace using .json format
   */
  void loadPaths(const std::string open_file);

  /**
   *  @brief save paths file
   */
  void savePaths();

  /**
   *  @brief set the color of path with some index
   *  @param index  the index of the path to set color
   *  @param color  the color to set
   */
  void setPathColor(const int& index, const QColor& color);

  /**
   *  @brief set the select status of path with some index
   *  @param index  the index of the path to set select status
   *  @param select   whether to select and visualize the path or not
   */
  void setPathSelectStatus(const int& index, const bool& select);

  /**
   *  @brief remove the path with some index from table view
   *  @param index  the index of the path to remove
   */
  void removePath(const int& index);

  void refresh();

Q_SIGNALS:
  /**
   *  @brief the signal informs values start_x_, start_y_, goal_x_, goal_y_ changed
   */
  void valueChanged();

protected:
  /**
   *  @brief update the start point, it is a callback funciton
   *  @param  pose    the start setting in Rviz
   */
  void _onStartUpdate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);

  /**
   *  @brief update the goal point, it is a callback funciton
   *  @param  pose    the goal setting in Rviz
   */
  void _onGoalUpdate(const geometry_msgs::PoseStamped::ConstPtr& pose);

  /**
   *  @brief if clicked signal is received, call this slot function
   */
  void _onClicked();

  /**
   *  @brief if editing finished signal is received, call this slot function
   */
  void _onEditingFinished();

public:
  ros::Publisher marker_pub_;  // map marker publisher
  ros::Publisher paths_pub_;   // paths publisher

  // start and goal point subscriber
  ros::Subscriber start_sub_, goal_sub_;

  // call plan client
  ros::ServiceClient call_plan_client_;

  // valid planner name list
  std::vector<std::string> planner_list_;

  // start and goal point
  Point2D start_, goal_;

  PathList* path_list_;
};
}  // namespace path_visual_plugin
#endif  // CORE_PATH_VISUAL_PLUGIN_H
