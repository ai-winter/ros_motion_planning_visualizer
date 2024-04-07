/**
 * *********************************************************
 *
 * @file: path_list.cpp
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
#include "path_visualizer/path_list.h"

namespace rmpv
{
/**
 * @brief Construct a new PathList object
 */
PathList::PathList()
{
  paths_.clear();
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
 * @return true if append successfully
 */
bool PathList::append(const PathInfo& path)
{
  if (size() >= MAX_PATH_NUM)
    return false;

  paths_.push_back(path);

  return true;
}

/**
 * @brief remove the path with some index from path list
 * @param index the index of path to remove
 * @return true if remove successfully
 */
bool PathList::remove(const int& index)
{
  if (index < 0 || index >= size())
    return false;

  paths_.erase(paths_.begin() + index);

  return true;
}

/**
 * @brief set the color of path with some index
 * @param index the index of path to set color
 * @param color the color to set
 * @return true if set successfully
 */
bool PathList::setColor(const int& index, const QColor& color)
{
  if (index < 0 || index >= size())
    return false;

  return paths_[index].setData(PathInfo::pathColor, color);
}

/**
 * @brief set the select status of path with some index
 * @param index the index of path to set select status
 * @param select  whether to select and visualize the path or not
 * @return true if set successfully
 */
bool PathList::setSelect(const int& index, const bool& select)
{
  if (index < 0 || index >= size())
    return false;

  return paths_[index].setData(PathInfo::selectStatus, select);
}

const QList<PathInfo>* PathList::getListPtr() const
{
  return &paths_;
}

/**
 * @brief save the selected paths to a local JSON file
 * @param file_name  the file name to save
 * @return true if save successfully
 */
bool PathList::save(QString file_name) const
{
  QJsonArray paths_array;

  for (const auto& info : paths_)
  {
    // save only the selected paths
    if (!info.getData(PathInfo::selectStatus).toBool())
      continue;

    QJsonObject path_json;

    // save the path info to JSON object
    path_json["planner"] = info.getData(PathInfo::plannerName).toString();

    QJsonObject start_json;
    start_json["x"] = info.getData(PathInfo::startPoseX).toDouble();
    start_json["y"] = info.getData(PathInfo::startPoseY).toDouble();
    start_json["theta"] = info.getData(PathInfo::startPoseTheta).toDouble();
    path_json["start"] = start_json;

    QJsonObject goal_json;
    goal_json["x"] = info.getData(PathInfo::goalPoseX).toDouble();
    goal_json["y"] = info.getData(PathInfo::goalPoseY).toDouble();
    goal_json["theta"] = info.getData(PathInfo::goalPoseTheta).toDouble();
    path_json["goal"] = goal_json;

    QJsonArray points_array;
    for (const auto& p : info.getPathPoints())
    {
      QJsonObject points;
      points["x"] = p.x;
      points["y"] = p.y;
      points_array.append(points);
    }
    path_json["path"] = points_array;

    QJsonObject color;
    QColor path_color = info.getData(PathInfo::pathColor).value<QColor>();
    color["R"] = path_color.red();
    color["G"] = path_color.green();
    color["B"] = path_color.blue();
    path_json["color"] = color;

    paths_array.append(path_json);
  }

  // save the paths to the file
  QFile file(file_name);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
  {
    ROS_ERROR("Failed to create and open file when saving paths!");
    return false;
  }
  QJsonObject root;
  root["paths"] = paths_array;
  QJsonDocument document;
  document.setObject(root);
  file.write(document.toJson());
  file.close();

  return true;
}

/**
 * @brief load the paths from a local JSON file
 * @param file_name  the file name to load
 * @return true if load successfully
 */
bool PathList::load(QString file_name)
{
  QFile file(file_name);
  if (!file.open(QIODevice::ReadOnly))
  {
    ROS_ERROR("Failed to open file when loading paths!");
    return false;
  }
  QByteArray file_content = file.readAll();
  file.close();

  QJsonParseError json_error;
  QJsonDocument document = QJsonDocument::fromJson(file_content, &json_error);
  if (json_error.error != QJsonParseError::NoError)
  {
    ROS_ERROR("Failed to parse JSON file when loading paths!");
    return false;
  }

  QJsonObject root = document.object();
  QJsonArray paths_array = root["paths"].toArray();
  for (const auto& p : paths_array)
  {
    QJsonObject path_json = p.toObject();
    QJsonObject start = path_json["start"].toObject();
    QJsonObject goal = path_json["goal"].toObject();
    QJsonArray points_array = path_json["path"].toArray();
    QList<Point2D> path_points;
    path_points.clear();
    for (const auto& point : points_array)
      path_points.push_back(Point2D(point.toObject()["x"].toDouble(), point.toObject()["y"].toDouble()));
    QJsonObject color = path_json["color"].toObject();

    // construct a new PathInfo object with the parsed info
    PathInfo path(
        path_json["planner"].toString(),
        Pose2D(start["x"].toDouble(), start["y"].toDouble(), start["theta"].toDouble()),
        Pose2D(goal["x"].toDouble(), goal["y"].toDouble(), goal["theta"].toDouble()),
        path_points,
        QColor(color["R"].toInt(), color["G"].toInt(), color["B"].toInt())
    );

    // add the path to path list
    append(path);
  }

  return true;
}

/**
 * @brief return the size of path list
 * @return the size of path list
 */
int PathList::size() const
{
  return paths_.size();
}
}  // namespace rmpv
