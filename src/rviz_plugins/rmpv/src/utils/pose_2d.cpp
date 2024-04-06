/**
 * *********************************************************
 *
 * @file: pose_2d.cpp
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
#include "utils/pose_2d.h"

namespace rmpv
{
/**
 * @brief Construct a new Pose2D object
 * @param x  the x coordinate of pose
 * @param y  the y coordinate of pose
 * @param theta the theta of pose
 */
Pose2D::Pose2D(double x, double y, double theta) : x(x), y(y), theta(theta)
{
  normalizeTheta();
}

/**
 * @brief Destroy the Pose2D object
 */
Pose2D::~Pose2D()
{
}

/**
 * @brief normalize theta to be within the range [-π, π]
 */
void Pose2D::normalizeTheta()
{
  double pi = std::acos(-1);

  theta = std::fmod(theta, 2.0 * pi);  // get the remainder of theta / (2*pi)
  if (theta > pi)
    theta -= 2.0 * pi;
  else if (theta < -pi)
    theta += 2.0 * pi;
}
}  // namespace rmpv