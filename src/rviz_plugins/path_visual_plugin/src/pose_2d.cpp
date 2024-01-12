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
#include "include/pose_2d.h"

namespace path_visual_plugin
{
/**
 * @brief Construct a new Pose2D object
 * @param x  the x coordinate of pose
 * @param y  the y coordinate of pose
 * @param yaw the yaw of pose
 */
Pose2D::Pose2D(double x, double y, double yaw): x(x), y(y), yaw(yaw){
  normalizeYaw();
}

/**
 * @brief Destroy the Pose2D object
 */
Pose2D::~Pose2D(){}

/**
 * @brief normalize yaw to be within the range [-π, π]
 */
void Pose2D::normalizeYaw() {
  double pi = std::acos(-1);

  yaw = std::fmod(yaw, 2.0 * pi); // get the remainder of yaw / (2*pi)
  if (yaw > pi)
    yaw -= 2.0 * pi;
  else if (yaw < -pi)
    yaw += 2.0 * pi;
}
}  // namespace path_visual_plugin
