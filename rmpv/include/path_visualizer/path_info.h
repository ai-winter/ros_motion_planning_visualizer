/**
 * *********************************************************
 *
 * @file: path_info.h
 * @brief: Contains PathInfo class
 * @author: Wu Maojia, Yang Haodong
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
#ifndef PATH_INFO_H
#define PATH_INFO_H

#include <cmath>

#include <QColor>
#include <QString>
#include <QList>
#include <QVariant>

#include "utils/point_2d.h"
#include "utils/pose_2d.h"

namespace rmpv
{
class PathInfo
{
public:
  enum { plannerName, startPose, startPoseX, startPoseY, startPoseTheta, goalPose, goalPoseX, goalPoseY,
    goalPoseTheta, pathLength, pathColor, turningAngle, selectStatus };

public:
  /**
   * @brief Construct a new PathInfo object with parameters
   * @param p_name  the name of planner
   * @param s  the start pose
   * @param g  the goal pose
   * @param pts  the path points
   * @param c  the color of path
   * @param slt  whether the path is selected
   */
  PathInfo(QString p_name = "None", Pose2D s = Pose2D(0.0, 0.0, 0.0), Pose2D g = Pose2D(0.0, 0.0, 0.0),
           QList<Point2D> pts = QList<Point2D>(), QColor c = Qt::darkBlue, bool slt = true);

  /**
   * @brief Destroy the PathInfo object
   */
  ~PathInfo();

  /*
   * @brief get some data from path info
   * @param variant  specify the data to get
   * @return the desired data
   */
  QVariant getData(const int& variant) const;

  /*
   * @brief get path points data from path info
   * @return the list of path points
   */
  QList<Point2D> getPathPoints() const;

  /*
   * @brief set some data of path info
   * @param variant  specify the data to set
   * @return true if set successfully
   */
  bool setData(const int& variant, const QVariant& value);

protected:

  /*
   * @brief Calculate path length
   * @return path length
   */
  double _calcPathLength();

  /*
   * @brief Calculate path turning angle
   * @return path turning angle
   */
  double _calcTurningAngle();

private:
  // the name of planner
  QString planner_name_;

  // start and goal pose
  Pose2D start_, goal_;

  // path planned
  QList<Point2D> path_;

  // path length and total turning angle
  double length_, turning_angle_;

  // the color of visualized path
  QColor color_;

  // select and visualize the path
  bool select_;
};
}  // namespace rmpv
#endif  // PATH_INFO_H
