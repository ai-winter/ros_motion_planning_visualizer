/***********************************************************
*
* @file: path_list.cpp
* @breif: Contains PathList class
* @author: Yang Haodong, Wu Maojia
* @update: 2023-10-14
* @version: 1.0
*
* Copyright (c) 2023， Yang Haodong, Wu Maojia
* All rights reserved.
* --------------------------------------------------------
*
**********************************************************/
#include "include/path_list.h"

namespace path_visual_plugin
{
/**
 * @brief Construct a new PathList object
 */
PathList::PathList()
{
  path_num_ = 0;
}

/**
 * @brief Destroy the PathList object
 */
PathList::~PathList()
{
}

/**
 * @brief append a new path to the path list
 * @param path  the new path to append
 */
bool PathList::append(PathInfo path)
{
  if (path_num_ >= MAX_PATH_NUM)
    return false;

  path_info_[path_num_] = path;
  ++path_num_;

  return true;
}

/**
 * @brief remove the path with some index from path list
 * @param index the index of path to remove
 */
bool PathList::remove(const int& index)
{
  if (index < 0 || index >= path_num_)
    return false;

  for (int i = index; i < path_num_ - 1; ++i)
    path_info_[i] = path_info_[i + 1];
  --path_num_;

  return true;
}

/**
 * @brief set the color of path with some index
 * @param index the index of path to set color
 * @param color the color to set
 */
bool PathList::setColor(const int& index, const QColor& color)
{
  if (index < 0 || index >= path_num_)
    return false;

  path_info_[index].color = color;

  return true;
}

/**
 * @brief set the show status of path with some index
 * @param index the index of path to set show status
 * @param show  whether to show the path or not
 */
bool PathList::setShow(const int& index, const bool& show)
{
  if (index < 0 || index >= path_num_)
    return false;

  path_info_[index].show = show;

  return true;
}

/**
 * @brief query the path info with some index
 * @param path  the variable that stores queried value
 * @param index the index of path to query
 */
bool PathList::query(PathInfo& path, const int& index)
{
  if (index < 0 || index >= path_num_)
    return false;

  path = path_info_[index];

  return true;
}

/**
 * @brief return the size of path list
 */
int PathList::size()
{
  return path_num_;
}
}  // namespace path_visual_plugin
