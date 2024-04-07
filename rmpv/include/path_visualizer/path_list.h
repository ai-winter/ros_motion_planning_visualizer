/**
 * *********************************************************
 *
 * @file: path_list.h
 * @brief: Contains PathList class
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
#ifndef PATH_LIST_H
#define PATH_LIST_H

#define MAX_PATH_NUM 10000  // the max number of paths in path list

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <ros/ros.h>

#include "path_visualizer/path_info.h"

namespace rmpv
{
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

  /**
   * @brief get the pointer of path info list
   * @return the pointer of path info list
   */
  const QList<PathInfo>* getListPtr() const;

  /**
   * @brief save the selected paths to a local JSON file
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
  QList<PathInfo> paths_; // the path list
};
}  // namespace rmpv
#endif  // PATH_LIST_H
