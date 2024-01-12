/***********************************************************
 *
 * @file: path_info.h
 * @breif: Contains PathInfo class
 * @author: Yang Haodong, Wu Maojia
 * @update: 2023-11-2
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PATH_INFO_H
#define PATH_INFO_H

#include <cmath>

#include <QColor>
#include <QString>
#include <QList>
#include <QVariant>

#include "include/point_2d.h"
#include "include/pose_2d.h"

namespace path_visual_plugin
{
class PathInfo
{
public:
  enum { plannerName, startPose, startPoseX, startPoseY, startPoseYaw, goalPose, goalPoseX, goalPoseY,
    goalPoseYaw, pathLength, pathColor, turningAngle, selectStatus };

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
  QColor color;

  // select and visualize the path
  bool select;
};
}  // namespace path_visual_plugin
#endif  // PATH_INFO_H
