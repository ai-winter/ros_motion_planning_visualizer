/**
 * *********************************************************
 *
 * @file: pose_2d.h
 * @brief: Contains Pose2D struct
 * @author: Wu Maojia
 * @date: 2024-01-12
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong, Wu Maojia. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef POSE_2D_H
#define POSE_2D_H

#include <cmath>
#include <QObject>

namespace rmpv
{
class Pose2D
{
public:
  /**
   * @brief Construct a new Pose2D object
   * @param x  the x coordinate of pose
   * @param y  the y coordinate of pose
   * @param theta the theta of pose
   */
  Pose2D(double x = 0.0, double y = 0.0, double theta = 0.0);

  /**
   * @brief Destroy the Pose2D object
   */
  ~Pose2D();

  /**
   * @brief normalize theta to be within the range [-π, π]
   */
  void normalizeTheta();

public:
  double x, y, theta;  // the x and y coordinate and theta of pose
};
}  // namespace rmpv
Q_DECLARE_METATYPE(rmpv::Pose2D)
#endif  // POSE_2D_H
