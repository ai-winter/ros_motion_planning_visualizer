/***********************************************************
 *
 * @file: pose_2d.h
 * @breif: Contains Pose2D struct
 * @author: Wu Maojia
 * @update: 2024-1-9
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef POSE_2D_H
#define POSE_2D_H

#include <cmath>
#include <QObject>

namespace path_visual_plugin
{
class Pose2D
{
public:
  /**
   * @brief Construct a new Pose2D object
   * @param x  the x coordinate of pose
   * @param y  the y coordinate of pose
   * @param yaw the yaw of pose
   */
  Pose2D(double x = 0.0, double y = 0.0, double yaw = 0.0);

  /**
   * @brief Destroy the Pose2D object
   */
  ~Pose2D();

  /**
   * @brief normalize yaw to be within the range [-π, π]
   */
  void normalizeYaw();

public:
  double x, y, yaw;  // the x and y coordinate and yaw of pose
};
}  // namespace path_visual_plugin
Q_DECLARE_METATYPE(path_visual_plugin::Pose2D)
#endif  // POSE_2D_H
