/**
 * *********************************************************
 *
 * @file: curve_list.cpp
 * @brief: Contains CurveList class
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
#include "curve_visualizer/curve_list.h"

namespace rmpv
{
/**
 * @brief Construct a new CurveList object
 */
CurveList::CurveList()
{
  curves_.clear();
}

/**
 * @brief Destroy the CurveList object
 */
CurveList::~CurveList()
{
}

/**
 * @brief append a new curve to the curve list
 * @param curve  the new curve to append
 * @return true if append successfully
 */
bool CurveList::append(const CurveInfo& curve)
{
  if (size() >= MAX_CURVE_NUM)
    return false;

  curves_.push_back(curve);

  return true;
}

/**
 * @brief remove the curve with some index from curve list
 * @param index the index of curve to remove
 * @return true if remove successfully
 */
bool CurveList::remove(const int& index)
{
  if (index < 0 || index >= size())
    return false;

  curves_.erase(curves_.begin() + index);

  return true;
}

/**
 * @brief set the color of curve with some index
 * @param index the index of curve to set color
 * @param color the color to set
 * @return true if set successfully
 */
bool CurveList::setColor(const int& index, const QColor& color)
{
  if (index < 0 || index >= size())
    return false;

  return curves_[index].setData(CurveInfo::curveColor, color);
}

/**
 * @brief set the select status of curve with some index
 * @param index the index of curve to set select status
 * @param select  whether to select and visualize the curve or not
 * @return true if set successfully
 */
bool CurveList::setSelect(const int& index, const bool& select)
{
  if (index < 0 || index >= size())
    return false;

  return curves_[index].setData(CurveInfo::selectStatus, select);
}

const QList<CurveInfo>* CurveList::getListPtr() const
{
  return &curves_;
}

/**
 * @brief save the selected curves to a local JSON file
 * @param file_name  the file name to save
 * @return true if save successfully
 */
bool CurveList::save(QString file_name) const
{
  QJsonArray curves_array;

  for (const auto& info : curves_)
  {
    // save only the selected curves
    if (!info.getData(CurveInfo::selectStatus).toBool())
      continue;

    QJsonObject curve_json;

    // save the curve info to JSON object
    curve_json["type"] = info.getData(CurveInfo::curveType).toString();

    QJsonArray poses_array;
    for (const auto& p : info.getCurvePoses())
    {
      QJsonObject pose;
      pose["x"] = p.x;
      pose["y"] = p.y;
      pose["theta"] = p.theta;
      poses_array.append(pose);
    }
    curve_json["poses"] = poses_array;

    QJsonArray points_array;
    for (const auto& p : info.getCurvePoints())
    {
      QJsonObject points;
      points["x"] = p.x;
      points["y"] = p.y;
      points_array.append(points);
    }
    curve_json["curve"] = points_array;

    QJsonObject color;
    QColor curve_color = info.getData(CurveInfo::curveColor).value<QColor>();
    color["R"] = curve_color.red();
    color["G"] = curve_color.green();
    color["B"] = curve_color.blue();
    curve_json["color"] = color;

    curves_array.append(curve_json);
  }

  // save the curves to the file
  QFile file(file_name);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
  {
    ROS_ERROR("Failed to create and open file when saving curves!");
    return false;
  }
  QJsonObject root;
  root["curves"] = curves_array;
  QJsonDocument document;
  document.setObject(root);
  file.write(document.toJson());
  file.close();

  return true;
}

/**
 * @brief load the curves from a local JSON file
 * @param file_name  the file name to load
 * @return true if load successfully
 */
bool CurveList::load(QString file_name)
{
  QFile file(file_name);
  if (!file.open(QIODevice::ReadOnly))
  {
    ROS_ERROR("Failed to open file when loading curves!");
    return false;
  }
  QByteArray file_content = file.readAll();
  file.close();

  QJsonParseError json_error;
  QJsonDocument document = QJsonDocument::fromJson(file_content, &json_error);
  if (json_error.error != QJsonParseError::NoError)
  {
    ROS_ERROR("Failed to parse JSON file when loading curves!");
    return false;
  }

  QJsonObject root = document.object();
  QJsonArray curves_array = root["curves"].toArray();
  for (const auto& p : curves_array)
  {
    QJsonObject curve_json = p.toObject();
    QJsonObject start = curve_json["start"].toObject();
    QJsonObject goal = curve_json["goal"].toObject();
    QJsonArray poses_array = curve_json["poses"].toArray();
    QJsonArray points_array = curve_json["curve"].toArray();

    QList<Pose2D> curve_poses;
    curve_poses.clear();
    for (const auto& pose : poses_array)
      curve_poses.push_back(Pose2D(pose.toObject()["x"].toDouble(), pose.toObject()["y"].toDouble(), pose.toObject()["theta"].toDouble()));

    QList<Point2D> curve_points;
    curve_points.clear();
    for (const auto& point : points_array)
      curve_points.push_back(Point2D(point.toObject()["x"].toDouble(), point.toObject()["y"].toDouble()));

    QJsonObject color = curve_json["color"].toObject();

    // construct a new CurveInfo object with the parsed info
    CurveInfo curve(
        curve_json["type"].toString(),
        curve_poses,
        curve_points,
        QColor(color["R"].toInt(), color["G"].toInt(), color["B"].toInt())
    );

    // add the curve to curve list
    append(curve);
  }

  return true;
}

/**
 * @brief return the size of curve list
 * @return the size of curve list
 */
int CurveList::size() const
{
  return curves_.size();
}
}  // namespace rmpv
