/***********************************************************
 *
 * @file: path_list.h
 * @breif: Contains PathInfo struct and PathList class
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

#include <QColor>

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
  /**
   * @brief Construct a new PathInfo object with parameters
   * @param p_name  the name of planner
   * @param s  the start point
   * @param g  the goal point
   * @param pts  the path points
   * @param c  the color of path
   * @param slt  whether the path is selected
   */
  PathInfo(std::string p_name = "None", Point2D s = Point2D(0.0, 0.0), Point2D g = Point2D(0.0, 0.0),
           std::vector<Point2D> pts = std::vector<Point2D>(), QColor c = Qt::darkBlue, bool slt = true);

  /**
   * @brief Destroy the PathInfo object
   */
  ~PathInfo();

  /**
   * @brief get the planner name of path info
   * @return the planner name
   */
  std::string getPlannerName();

  /**
   * @brief get the start point of path info
   * @return the start point
   */
  Point2D getStart();

  /**
   * @brief get the goal point of path info
   * @return the goal point
   */
  Point2D getGoal();

  /**
   * @brief get the path points of path info
   * @return the path points
   */
  std::vector<Point2D> getPathPoints();

  /**
   * @brief get the length of path info
   * @return the length of path
   */
  double getLength();

  /**
   * @brief get the turning angle of path info
   * @return the turning angle of path
   */
  double getTurningAngle();

public:
  // the color of visualized path
  QColor color;

  // select and visualize the path
  bool select;

private:
  // the name of planner
  std::string planner_name_;

  // start and goal point
  Point2D start_, goal_;

  // path planned
  std::vector<Point2D> path_;

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
  bool append(PathInfo path);

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

  /**
   * @brief query the path info with some index
   * @param path  the variable that stores queried value
   * @param index the index of path to query
   * @return true if query successfully
   */
  bool query(PathInfo& path, const int& index);

  /**
   * @brief save the paths to a local JSON file
   * @param file_name  the file name to save
   * @return true if save successfully
   */
  bool save(std::string file_name);

  /**
   * @brief load the paths from a local JSON file
   * @param file_name  the file name to load
   * @return true if load successfully
   */
  bool load(std::string file_name);

  /**
   * @brief return the size of path list
   * @return the size of path list
   */
  int size();

private:
  std::vector<PathInfo> path_info_; // the path list
};
}  // namespace path_visual_plugin
#endif  // PATH_LIST_H
