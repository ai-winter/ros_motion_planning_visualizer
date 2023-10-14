/***********************************************************
 *
 * @file: path_list.h
 * @breif: Contains PathInfo struct and PathList class
 * @author: Yang Haodong, Wu Maojia
 * @update: 2023-10-14
 * @version: 1.0
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

namespace path_visual_plugin
{
  struct PathInfo
  {
  public:
    // the name of planner
    std::string planner_name;

    // start and goal point
    double start_x, start_y, goal_x, goal_y;

    // path length and total turning angle
    double length, turning_angle;

    // the color of visualized path
    QColor color;

    // whether to visualize the path
    bool show;
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
     */
    bool append(PathInfo path);

    /**
     * @brief remove the path with some index from path list
     * @param index the index of path to remove
     */
    bool remove(const int& index);

    /**
     * @brief set the color of path with some index
     * @param index the index of path to set color
     * @param color the color to set
     */
    bool setColor(const int& index, const QColor& color);

    /**
     * @brief set the show status of path with some index
     * @param index the index of path to set show status
     * @param show  whether to show the path or not
     */
    bool setShow(const int& index, const bool& show);

    /**
     * @brief query the path info with some index
     * @param path  the variable that stores queried value
     * @param index the index of path to query
     */
    bool query(PathInfo& path, const int& index);

    /**
     * @brief return the size of path list
     */
    int size();

  private:
    PathInfo path_info_[MAX_PATH_NUM + 10];

    int path_num_;
  };
}  // namespace path_visual_plugin
#endif  // PATH_LIST_H
