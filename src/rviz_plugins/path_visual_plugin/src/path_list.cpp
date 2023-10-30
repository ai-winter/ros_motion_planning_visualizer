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
#include "include/path_list.h"

namespace path_visual_plugin
{
/**
 * @brief Construct a new PathInfo object
 * @param p_name  the name of planner
 * @param s  the start point
 * @param g  the goal point
 * @param pts  the path points
 * @param c  the color of path
 * @param slt  whether the path is selected
 */
PathInfo::PathInfo(QString p_name, Point2D s, Point2D g, QList<Point2D> pts, QColor c, bool slt)
{
  planner_name_ = p_name;
  start_ = s;
  goal_ = g;
  path_ = pts;
  color = c;
  select = slt;
}

/**
 * @brief Destroy the PathInfo object
 */
PathInfo::~PathInfo()
{
}

QVariant PathInfo::getData(const int &variant) const
{
  switch (variant)
  {
    case plannerName:
      return planner_name_;
    case startPoint:
      return QVariant::fromValue(start_);
    case startPointX:
      return start_.x;
    case startPointY:
      return start_.y;
    case goalPoint:
      return QVariant::fromValue(goal_);
    case goalPointX:
      return goal_.x;
    case goalPointY:
      return goal_.y;
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

QList<Point2D> PathInfo::getPathPoints() const
{
  return path_;
}

/**
 * @brief Construct a new PathList object
 */
PathList::PathList()
{
  path_info_.clear();
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
 * @return true if remove successfully
 */
bool PathList::remove(const int& index)
{
  if (index < 0 || index >= size())
    return false;

  path_info_.erase(path_info_.begin() + index);

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

  path_info_[index].color = color;

  return true;
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

  path_info_[index].select = select;

  return true;
}

/**
 * @brief query the path info with some index
 * @param path  the variable that stores queried value
 * @param index the index of path to query
 * @return true if query successfully
 */
bool PathList::query(PathInfo& path, const int& index)
{
  if (index < 0 || index >= size())
    return false;

  path = path_info_[index];

  return true;
}

//bool PathList::getSelect(const int& index)
//{
//  if (index < 0 || index >= size())
//    return false;
//
//  return path_info_[index].select;
//}

/**
 * @brief save the paths to a local JSON file
 * @param file_name  the file name to save
 * @return true if save successfully
 */
bool PathList::save(std::string file_name)
{
  QJsonArray paths_array;

  for (int i = 0; i < size(); i++)
  {
    QJsonObject path_json;
    PathInfo path;

    // query the i-th path info
    if (query(path, i))
    {
      // save the path info to JSON object
      path_json["planner"] = path.getData(PathInfo::plannerName).toString();

      QJsonObject start_json;
      start_json["x"] = path.getData(PathInfo::startPointX).toDouble();
      start_json["y"] = path.getData(PathInfo::startPointY).toDouble();
      path_json["start"] = start_json;

      QJsonObject goal_json;
      goal_json["x"] = path.getData(PathInfo::goalPointX).toDouble();
      goal_json["y"] = path.getData(PathInfo::goalPointY).toDouble();
      path_json["goal"] = goal_json;

      QJsonArray points_array;
      for (const auto p : path.getPathPoints())
      {
        QJsonObject points;
        points["x"] = p.x;
        points["y"] = p.y;
        points_array.append(points);
      }
      path_json["path"] = points_array;

      QJsonObject color;
      color["R"] = path.color.red();
      color["G"] = path.color.green();
      color["B"] = path.color.blue();
      path_json["color"] = color;

      paths_array.append(path_json);
    }
    else
    {
      ROS_ERROR("Failed to query path info when saving paths!");
      return false;
    }
  }

  std::ofstream file(file_name);

  // save the paths to the file
  if (file.is_open())
  {
    QJsonObject root;
    root["paths"] = paths_array;
    QJsonDocument document;
    document.setObject(root);
    file << document.toJson().toStdString();
    file.close();
  }
  else
  {
    ROS_ERROR("Failed to create and open file when saving paths!");
    return false;
  }

  return true;
}

/**
 * @brief load the paths from a local JSON file
 * @param file_name  the file name to load
 * @return true if load successfully
 */
bool PathList::load(std::string file_name)
{
  std::ifstream infile(file_name, std::ios::in);
  if (infile.is_open())
  {
    // read the whole file content
    std::stringstream buffer;
    buffer << infile.rdbuf();
    std::string infileContent = buffer.str();

    // parse the file content to JSON object
    QJsonParseError json_error;
    QJsonDocument document = QJsonDocument::fromJson(QString::fromStdString(infileContent).toUtf8(), &json_error);
    if (json_error.error == QJsonParseError::NoError)
    {
      QJsonObject root = document.object();
      QJsonArray paths_array = root["paths"].toArray();
      for (const auto p : paths_array)
      {
        QJsonObject path_json = p.toObject();
        QJsonObject start = path_json["start"].toObject();
        QJsonObject goal = path_json["goal"].toObject();
        QJsonArray points_array = path_json["path"].toArray();
        QList<Point2D> path_points;
        path_points.clear();
        for (const auto point : points_array)
          path_points.push_back(Point2D(point.toObject()["x"].toDouble(), point.toObject()["y"].toDouble()));
        QJsonObject color = path_json["color"].toObject();

        // construct a new PathInfo object with the parsed info
        PathInfo path(
            path_json["planner"].toString(),
            Point2D(start["x"].toDouble(), start["y"].toDouble()),
            Point2D(goal["x"].toDouble(), goal["y"].toDouble()),
            path_points,
            QColor(color["R"].toInt(), color["G"].toInt(), color["B"].toInt())
            );

        // add the path to path list
        append(path);
      }
    }
    else
    {
      ROS_ERROR("Failed to parse JSON file when loading paths!");
      return false;
    }
  }
  else
  {
    ROS_ERROR("Failed to open file when loading paths!");
    return false;
  }

  return true;
}

/**
 * @brief return the size of path list
 */
int PathList::size()
{
  return path_info_.size();
}
}  // namespace path_visual_plugin
