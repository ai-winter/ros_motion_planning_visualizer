/***********************************************************
 *
 * @file: path_info.cpp
 * @breif: Contains PathInfo class
 * @author: Yang Haodong, Wu Maojia
 * @update: 2024-1-9
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "include/path_info.h"

namespace path_visual_plugin
{
/**
 * @brief Construct a new PathInfo object
 * @param p_name  the name of planner
 * @param s  the start pose
 * @param g  the goal pose
 * @param pts  the path points
 * @param c  the color of path
 * @param slt  whether the path is selected
 */
PathInfo::PathInfo(QString p_name, Pose2D s, Pose2D g, QList<Point2D> pts, QColor c, bool slt)
{
  planner_name_ = p_name;
  start_ = s;
  goal_ = g;
  path_ = pts;
  color = c;
  select = slt;

  length_ = _calcPathLength();
  turning_angle_ = _calcTurningAngle();
}

/**
 * @brief Destroy the PathInfo object
 */
PathInfo::~PathInfo()
{
}

/*
 * @brief get some data from path info
 * @param variant  specify the data to get
 * @return the desired data
 */
QVariant PathInfo::getData(const int& variant) const
{
  switch (variant)
  {
    case plannerName:
      return planner_name_;
    case startPose:
      return QVariant::fromValue(start_);
    case startPoseX:
      return start_.x;
    case startPoseY:
      return start_.y;
    case startPoseYaw:
      return start_.yaw;
    case goalPose:
      return QVariant::fromValue(goal_);
    case goalPoseX:
      return goal_.x;
    case goalPoseY:
      return goal_.y;
    case goalPoseYaw:
      return goal_.yaw;
    case pathLength:
      return length_;
    case pathColor:
      return color;
    case turningAngle:
      return turning_angle_;
    case selectStatus:
      return select;
    default:
      return QVariant();
  }
}

/*
 * @brief get path points data from path info
 * @return the list of path points
 */
QList<Point2D> PathInfo::getPathPoints() const
{
  return path_;
}

/*
 * @brief set some data of path info
 * @param variant  specify the data to set
 * @return true if set successfully
 */
bool PathInfo::setData(const int& variant, const QVariant& value)
{
  switch (variant)
  {
    case pathColor:
      color = value.value<QColor>();
      break;
    case selectStatus:
      select = value.toBool();
      break;
    default:
      return false;
  }
  return true;
}

/*
 * @brief Calculate path length
 * @return path length
 */
double PathInfo::_calcPathLength()
{
  double length = 0.0;
  for (int i = 0; i < path_.size() - 1; i++)
    length += std::sqrt(std::pow(path_[i + 1].x - path_[i].x, 2) + std::pow(path_[i + 1].y - path_[i].y, 2));

  return length;
}

/*
 * @brief Calculate path turning angle
 * @return path turning angle
 */
double PathInfo::_calcTurningAngle()
{
  double angle = 0.0;
  for (int i = 0; i < path_.size() - 2; i++)
  {
    double x1 = path_[i + 1].x - path_[i].x;
    double y1 = path_[i + 1].y - path_[i].y;
    double x2 = path_[i + 2].x - path_[i + 1].x;
    double y2 = path_[i + 2].y - path_[i + 1].y;
    angle += std::acos(
        std::max(-1.0, std::min(1.0,
                              (x1 * x2 + y1 * y2) / (std::sqrt(x1 * x1 + y1 * y1) * std::sqrt(x2 * x2 + y2 * y2))
                              )));
  }
  // take yaw angle into account
  angle += std::abs(start_.yaw - goal_.yaw);
  return angle;
}
}  // namespace path_visual_plugin
