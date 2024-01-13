/***********************************************************
 *
 * @file: core_curve_visualizer.h
 * @breif: Contains core of curve visualizer class
 * @author: Wu Maojia, Yang Haodong
 * @update: 2024-1-12
 * @version: 1.0
 *
 * Copyright (c) 2024， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef CORE_CURVE_VISUALIZER_H
#define CORE_CURVE_VISUALIZER_H

#include <rviz/panel.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include "wrapper_planner/CallPlan.h"

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

public:
};
}  // namespace rmpv
#endif  // CORE_CURVE_VISUALIZER_H
