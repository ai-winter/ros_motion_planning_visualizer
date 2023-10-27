/***********************************************************
 *
 * @file: path_list.cpp
 * @breif: Contains PathList class
 * @author: Yang Haodong, Wu Maojia
 * @update: 2023-10-27
 * @version: 2.0
 *
 * Copyright (c) 2023， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <json/json.h>

#include <ros/ros.h>

#include "include/path_list.h"

namespace path_visual_plugin
{
PathInfo::PathInfo()
  : planner_name("None"), start_x(0.0), start_y(0.0), goal_x(0.0), goal_y(0.0), color(Qt::darkBlue), show(true)
{
}

PathInfo::PathInfo(std::string p_name, double s_x, double s_y, double g_x, double g_y, QColor c, bool is_show)
{
  planner_name = p_name;
  start_x = s_x;
  start_y = s_y;
  goal_x = g_x;
  goal_y = g_y;
  color = c;
  show = is_show;
}

void PathInfo::setPath(std::vector<geometry_msgs::PoseStamped>& ros_path)
{
  path.clear();
  for (const auto p : ros_path)
    path.push_back(Point2D{ x : p.pose.position.x, y : p.pose.position.y });
}

/**
 * @brief Construct a new PathList object
 */
PathList::PathList()
{
  path_info_.clear();
  // path_num_ = 0;
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
  if (size() >= MAX_PATH_NUM)
    return false;

  path_info_.push_back(path);

  return true;
}

/**
 * @brief remove the path with some index from path list
 * @param index the index of path to remove
 */
bool PathList::remove(const int& index)
{
  if (index < 0 || index >= size())
    return false;

  std::swap(*(std::begin(path_info_) + index), path_info_.back());
  path_info_.pop_back();
  // TODO index error?

  return true;
}

/**
 * @brief set the color of path with some index
 * @param index the index of path to set color
 * @param color the color to set
 */
bool PathList::setColor(const int& index, const QColor& color)
{
  if (index < 0 || index >= size())
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
  if (index < 0 || index >= size())
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
  if (index < 0 || index >= size())
    return false;

  path = path_info_[index];

  return true;
}

/**
 * @brief save the paths to a local JSON file
 * @param file_name  the file name to save
 */
void PathList::save(std::string file_name)
{
  Json::Value root;

  for (int i = 0; i < size(); i++)
  {
    Json::Value path_json;
    PathInfo path;
    query(path, i);
    path_json["index"] = i;
    path_json["planner"] = path.planner_name;
    path_json["start_x"] = path.start_x;
    path_json["start_y"] = path.start_y;
    path_json["goal_x"] = path.goal_x;
    path_json["goal_y"] = path.goal_y;

    for (const auto p : path.path)
    {
      Json::Value points;
      points["x"] = p.x;
      points["y"] = p.y;
      path_json["path"].append(points);
    }

    Json::Value color;
    color["R"] = qRed(path.color.rgb());
    color["G"] = qGreen(path.color.rgb());
    color["B"] = qBlue(path.color.rgb());
    path_json["color"] = color;

    // TODO Length and angle

    root["paths"].append(path_json);
  }

  std::ofstream file(file_name);

  if (file.is_open())
  {
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(root, &file);
    file.close();
  }
}

/**
 * @brief load the paths from a local JSON file
 * @param file_name  the file name to load
 */
void PathList::load(std::string file_name)
{
  Json::Reader reader;
  Json::Value root;

  std::ifstream infile(file_name, std::ios::in);
  if (infile.is_open() && reader.parse(infile, root))
  {
    for (const auto p : root["paths"])
    {
      PathInfo path;
      path.planner_name = p["planner"].asString();
      path.start_x = p["start_x"].asDouble();
      path.start_y = p["start_y"].asDouble();
      path.goal_x = p["goal_x"].asDouble();
      path.goal_y = p["goal_y"].asDouble();

      for (const auto point : p["path"])
        path.path.push_back(Point2D{ x : point["x"].asDouble(), y : point["y"].asDouble() });

      path.color = QColor(p["color"]["R"].asInt(), p["color"]["G"].asInt(), p["color"]["B"].asInt());
      path.show = true;
      append(path);
    }
  }
}

/**
 * @brief return the size of path list
 */
int PathList::size()
{
  return path_info_.size();
}
}  // namespace path_visual_plugin
