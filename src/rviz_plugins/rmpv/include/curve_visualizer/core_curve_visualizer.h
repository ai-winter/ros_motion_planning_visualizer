/**
 * *********************************************************
 *
 * @file: core_curve_visualizer.h
 * @brief: Contains core of curve visualizer class
 * @author: Wu Maojia, Yang Haodong
 * @date: 2024-01-13
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef CORE_CURVE_VISUALIZER_H
#define CORE_CURVE_VISUALIZER_H

#include <rviz/panel.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "curve_visualizer/curve_list.h"

namespace rmpv
{
class CoreCurveVisualizer : public QObject
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new CoreCurveVisualizer object
   */
  CoreCurveVisualizer();

  /**
   * @brief Destroy the CoreCurveVisualizer object
   */
  ~CoreCurveVisualizer();

  /**
   * @brief ROS parameters initialization
   */
  void setupROS();

  /**
   *  @brief call curve generation service
   *  @param type type of curve
   */
  void addCurve(const QString& type);

  /**
   *  @brief save curves file
   *  @param save_file  save curves to local workspace using .json format
   */
  void saveCurves(const QString& save_file);

  /**
   *  @brief load curves file
   *  @param open_file  load curves from local workspace using .json format
   */
  void loadCurves(const QString open_file);

  /**
   *  @brief set the color of curve with some index
   *  @param index  the index of the curve to set color
   *  @param color  the color to set
   */
  void setCurveColor(const int& index, const QColor& color);

  /**
   *  @brief set the select status of curve with some index
   *  @param index  the index of the curve to set select status
   *  @param select   whether to select and visualize the curve or not
   */
  void setCurveSelectStatus(const int& index, const bool& select);

  /**
   *  @brief remove the curve with some index from table view
   *  @param index  the index of the curve to remove
   */
  void removeCurve(const int& index);

public:
  ros::Publisher marker_pub_;  // map marker publisher
  ros::Publisher curves_pub_;  // curves publisher
  ros::Publisher poses_pub_;   // poses publisher

  // curve info list
  CurveList* curve_list_;
};
}  // namespace rmpv
#endif  // CORE_CURVE_VISUALIZER_H
