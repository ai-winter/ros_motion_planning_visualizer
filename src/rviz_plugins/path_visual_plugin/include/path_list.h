/***********************************************************
 *
 * @file: path_list.h
 * @breif: Contains Point2D struct, PathInfo class and PathList class
 * @author: Yang Haodong, Wu Maojia
 * @update: 2023-10-27
 * @version: 2.0
 *
 * Copyright (c) 2023， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PATH_LIST_H
#define PATH_LIST_H

#define MAX_PATH_NUM 10000

#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <QColor>
#include <QString>
#include <QList>
#include <QVariant>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

namespace path_visual_plugin
{
struct Point2D
{
public:
  /**
   * @brief Construct a new Point2D object
   * @param x  the x coordinate of point
   * @param y  the y coordinate of point
   */
  Point2D(double x = 0.0, double y = 0.0): x(x), y(y) {};

public:
  double x, y;  // the x and y coordinate of point
};

class PathInfo
{
public:
  enum { plannerName, startPoint, startPointX, startPointY, goalPoint, goalPointX, goalPointY, pathLength, pathColor,
    turningAngle, selectStatus };

public:
  /**
   * @brief Construct a new PathInfo object with parameters
   * @param p_name  the name of planner
   * @param s  the start point
   * @param g  the goal point
   * @param pts  the path points
   * @param c  the color of path
   * @param slt  whether the path is selected
   */
  PathInfo(QString p_name = "None", Point2D s = Point2D(0.0, 0.0), Point2D g = Point2D(0.0, 0.0),
           QList<Point2D> pts = QList<Point2D>(), QColor c = Qt::darkBlue, bool slt = true);

  /**
   * @brief Destroy the PathInfo object
   */
  ~PathInfo();

  QVariant getData(const int& variant) const;

  QList<Point2D> getPathPoints() const;

public:
  // the color of visualized path
  QColor color;

  // select and visualize the path
  bool select;

private:
  // the name of planner
  QString planner_name_;

  // start and goal point
  Point2D start_, goal_;

  // path planned
  QList<Point2D> path_;

  // path length and total turning angle
  double length_, turning_angle_;
};

class PathList
{
public:
  /**
   * @brief Construct a new PathList object
   */
  PathList();

  /**
   * @brief Destroy the PathList object
   */
  ~PathList();

  /**
   * @brief append a new path to the path list
   * @param path  the new path to append
   * @return true if append successfully
   */
  bool append(const PathInfo& path);

  /**
   * @brief remove the path with some index from path list
   * @param index the index of path to remove
   * @return true if remove successfully
   */
  bool remove(const int& index);

  /**
   * @brief set the color of path with some index
   * @param index the index of path to set color
   * @param color the color to set
   * @return true if set successfully
   */
  bool setColor(const int& index, const QColor& color);

  /**
   * @brief set the select status of path with some index
   * @param index the index of path to set select status
   * @param select  whether to select and visualize the path or not
   * @return true if set successfully
   */
  bool setSelect(const int& index, const bool& select);

  const QList<PathInfo>* getListPtr() const;

  /**
   * @brief save the paths to a local JSON file
   * @param file_name  the file name to save
   * @return true if save successfully
   */
  bool save(QString file_name) const;

  /**
   * @brief load the paths from a local JSON file
   * @param file_name  the file name to load
   * @return true if load successfully
   */
  bool load(QString file_name);

  /**
   * @brief return the size of path list
   * @return the size of path list
   */
  int size() const;

private:
  QList<PathInfo> path_info_; // the path list
};
}  // namespace path_visual_plugin
Q_DECLARE_METATYPE(path_visual_plugin::Point2D)
#endif  // PATH_LIST_H
