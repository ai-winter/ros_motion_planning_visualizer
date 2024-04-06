/**
 * *********************************************************
 *
 * @file: core_path_visualizer.h
 * @brief: Contains core of path visualizer class
 * @author: Wu Maojia, Yang Haodong
 * @date: 2024-4-6
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong, Wu Maojia. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef CORE_PATH_VISUALIZER_H
#define CORE_PATH_VISUALIZER_H

#include <rviz/panel.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <nav_msgs/Path.h>

#include "wrapper_planner/CallPlan.h"
#include "path_visualizer/path_list.h"

namespace rmpv
{
class CorePathVisualizer : public QObject
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new CorePathVisualizer object
   */
  CorePathVisualizer();

  /**
   * @brief Destroy the CorePathVisualizer object
   */
  ~CorePathVisualizer();

  /**
   * @brief ROS parameters initialization
   */
  void setupROS();

  /**
   *  @brief call path planning service
   *  @param planner_name name of planner
   */
  void addPath(const QString& planner_name);

  /**
   *  @brief save paths file
   *  @param save_file  save paths to local workspace using .json format
   */
  void savePaths(const QString& save_file);

  /**
   *  @brief load paths file
   *  @param open_file  load paths from local workspace using .json format
   */
  void loadPaths(const QString open_file);

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

  /**
   *  @brief refresh paths displayed in rviz
   */
  void refresh_paths();

  /**
   *  @brief refresh start and goal poses displayed in rviz
   */
  void refresh_poses();

Q_SIGNALS:
  /**
   *  @brief the signal informs values start_x_, start_y_, goal_x_, goal_y_ changed
   */
  void valueChanged();

protected:
  /**
   *  @brief update the start pose marker, it is a callback funciton
   *  @param int_marker  interactive marker message
   */
  void _onStartUpdate(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& int_marker);

  /**
   *  @brief update the goal pose marker, it is a callback funciton
   *  @param int_marker  interactive marker message
   */
  void _onGoalUpdate(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& int_marker);

  /**
   * @brief make an interactive marker for pose
   * @param name  name of the marker
   * @param pose  pose of the marker
   * @return an interactive marker message
   */
  visualization_msgs::InteractiveMarker makeInteractiveMarker(const std::string& name, const Pose2D& pose);

  /**
   *  @brief if clicked signal is received, call this slot function
   */
  void _onClicked();

  /**
   *  @brief if editing finished signal is received, call this slot function
   */
  void _onEditingFinished();

public:
  ros::Publisher paths_pub_;   // paths publisher
  ros::Publisher start_pub_;   // start pose publisher
  ros::Publisher goal_pub_;    // goal pose publisher

  // start and goal point subscriber
  ros::Subscriber start_sub_, goal_sub_;

  // interactive marker server
  interactive_markers::InteractiveMarkerServer* im_server_;

  // call plan client
  ros::ServiceClient call_plan_client_;

  // valid planner name list
  std::vector<std::string> planner_list_;

  // start and goal point
  Pose2D start_, goal_;

  // path info list
  PathList* path_list_;
};
}  // namespace rmpv
#endif  // CORE_PATH_VISUALIZER_H
