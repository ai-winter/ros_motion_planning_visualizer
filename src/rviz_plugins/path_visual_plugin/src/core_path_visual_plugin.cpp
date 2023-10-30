/***********************************************************
 *
 * @file: core_path_visual_plugin.cpp
 * @breif: Contains core of path visualization Rviz plugin class
 * @author: Yang Haodong, Wu Maojia
 * @update: 2023-10-27
 * @version: 2.0
 *
 * Copyright (c) 2023， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#include <visualization_msgs/Marker.h>
#include "visualization_msgs/MarkerArray.h"
#include <nav_msgs/Path.h>

#include "wrapper_planner/CallPlan.h"
#include "include/core_path_visual_plugin.h"

namespace path_visual_plugin
{
/**
 * @brief Construct a new CorePathVisualPlugin object
 */
CorePathVisualPlugin::CorePathVisualPlugin() : path_list_(new PathList)
{
  start_ = goal_ = Point2D(0.0, 0.0);
}

/**
 * @brief Destroy the CorePathVisualPlugin object
 */
CorePathVisualPlugin::~CorePathVisualPlugin()
{
  if (path_list_)
    delete path_list_;
}

/**
 * @brief ROS parameters initialization
 */
void CorePathVisualPlugin::setupROS()
{
  // initialize ROS node
  ros::NodeHandle private_nh("");

  // publisher
  marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  paths_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("/paths", 10);

  // subscriber
  start_sub_ = private_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1, boost::bind(&CorePathVisualPlugin::_onStartUpdate, this, _1));
  goal_sub_ = private_nh.subscribe<geometry_msgs::PoseStamped>(
      "move_base_simple/goal", 1, boost::bind(&CorePathVisualPlugin::_onGoalUpdate, this, _1));

  // client
  call_plan_client_ = private_nh.serviceClient<wrapper_planner::CallPlan>("/move_base/WrapperPlanner/call_plan");

  // parameters
  private_nh.getParam("/move_base/planner", planner_list_);
}

/**
 *  @brief call path planning service
 */
void CorePathVisualPlugin::addPath(const QString& planner_name)
{
  geometry_msgs::PoseStamped start, goal;
  start.header.frame_id = "map";
  start.header.stamp = ros::Time::now();
  start.pose.position.x = start_.x;
  start.pose.position.y = start_.y;
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = goal_.x;
  goal.pose.position.y = goal_.y;

  wrapper_planner::CallPlan call_plan_srv;
  call_plan_srv.request.start = start;
  call_plan_srv.request.goal = goal;
  call_plan_srv.request.planner_name = planner_name.toStdString();

  ROS_WARN("call planner %s", planner_name.toStdString().c_str());
  ROS_WARN("start: %f, %f", start.pose.position.x, start.pose.position.y);
  ROS_WARN("goal: %f, %f", goal.pose.position.x, goal.pose.position.y);

  if (call_plan_client_.call(call_plan_srv))
  {
    // get path points from service response
    QList<Point2D> path_points;
    path_points.clear();
    for (const auto p : call_plan_srv.response.path)
      path_points.push_back(Point2D(p.pose.position.x, p.pose.position.y));

    // construct path info
    PathInfo path(planner_name, start_, goal_, path_points);

    if (path_list_->append(path))
    {
      refresh();
      ROS_INFO("Planner %s planning successfully done.", planner_name.toStdString().c_str());
    }
    else
    {
      ROS_ERROR("Planner %s planning failed.", planner_name.toStdString().c_str());
      return;
    }
  }
  else
  {
    ROS_ERROR("Planner %s planning failed.", planner_name.toStdString().c_str());
    return;
  }
}

/**
 *  @brief call load paths service
 *  @param open_file  load paths from local workspace using .json format
 */
void CorePathVisualPlugin::loadPaths(const std::string open_file)
{
  ROS_INFO("Loading path information at location %s", open_file.c_str());

  path_list_->load(open_file);
  refresh();
}

/**
 *  @brief call save paths service
 */
void CorePathVisualPlugin::savePaths()
{
  // TODO if valid_size > 0
  std::string cur_dir(std::getenv("PWD"));
  std::string save_dir = cur_dir + std::string("/../../user_data");

  if (access(save_dir.c_str(), F_OK) == -1)
    mkdir(save_dir.c_str(), S_IRWXU);

  int cnt = 0;
  DIR* dir_ptr = opendir(save_dir.c_str());
  struct dirent* dp = NULL;
  while ((dp = readdir(dir_ptr)) != NULL)
  {
    std::string f_name = dp->d_name;
    if (f_name == "." || f_name == "..")
      continue;

    if (dp->d_type == DT_REG)  // 文件
      cnt++;
  }
  closedir(dir_ptr);
  delete dp;

  std::ostringstream ostr;
  ostr << "/paths_" << cnt << ".json";
  std::string save_file = save_dir + ostr.str();

  ROS_INFO("Saving path information at location %s", save_file.c_str());

  path_list_->save(save_file);
}

/**
 *  @brief set the color of path with some index
 *  @param index  the index of the path to set color
 *  @param color  the color to set
 */
void CorePathVisualPlugin::setPathColor(const int& index, const QColor& color)
{
  if (path_list_->setColor(index, color))
  {
    QRgb color_rgb = color.rgb();
    ROS_INFO("The color of path with index %d is successfully set to RGB(%d, %d, %d)!", index, qRed(color_rgb),
             qGreen(color_rgb), qBlue(color_rgb));
    refresh();
  }
  else
    ROS_ERROR("Failed to set the color of path with index %d.", index);
}

/**
 *  @brief set the select status of path with some index
 *  @param index  the index of the path to set select status
 *  @param select   whether to select and visualize the path or not
 */
void CorePathVisualPlugin::setPathSelectStatus(const int& index, const bool& select)
{
  if (path_list_->setSelect(index, select))
  {
    ROS_INFO("The select status of path with index %d is successfully set to %s!", index, select ? "true" : "false");
    refresh();
  }
  else
    ROS_ERROR("Failed to set the select status of path with index %d.", index);
}

/**
 *  @brief remove the path with some index from table view
 *  @param index  the index of the path to remove
 */
void CorePathVisualPlugin::removePath(const int& index)
{
  if (path_list_->remove(index))
  {
    ROS_INFO("Path with index %d is successfully removed!", index);
  }
  else
    ROS_ERROR("Failed to remove path with index %d.", index);
}

void CorePathVisualPlugin::refresh()
{
  // point_line_pub.publish(MarkerArray);
  visualization_msgs::Marker path_marker;
  visualization_msgs::MarkerArray path_marker_array;
  int marker_id = 0;
  for (int i = 0; i < path_list_->size(); i++)
  {
    PathInfo path;
    if (path_list_->query(path, i))
    {
      QList<Point2D> path_points = path.getPathPoints();
      for (unsigned int j = 0; j < path_points.size() - 1; j++)
      {
        path_marker.ns = "path_marker";
        path_marker.type = visualization_msgs::Marker::LINE_LIST;
        path_marker.action = path_marker.ADD;
        geometry_msgs::Point p;
        p.x = path_points[j].x;
        p.y = path_points[j].y;
        p.z = 0.0;
        path_marker.points.push_back(p);
        p.x = path_points[j + 1].x;
        p.y = path_points[j + 1].y;
        p.z = 0.0;
        path_marker.points.push_back(p);

        path_marker.pose.orientation.x = 0.0;
        path_marker.pose.orientation.y = 0.0;
        path_marker.pose.orientation.z = 0.0;
        path_marker.pose.orientation.w = 1.0;
        path_marker.scale.x = 0.1;

        //设置线的颜色，a应该是透明度
        path_marker.color.r = qRed(path.color.rgb()) / 255.0;
        path_marker.color.g = qGreen(path.color.rgb()) / 255.0;
        path_marker.color.b = qBlue(path.color.rgb()) / 255.0;
        path_marker.color.a = 1.0;

        path_marker.lifetime = ros::Duration();
        path_marker.id = marker_id;

        path_marker.header.frame_id = "map";
        path_marker.header.stamp = ros::Time::now();
        path_marker_array.markers.push_back(path_marker);
        path_marker.points.clear();

        marker_id++;
      }
    }
    else
      ROS_ERROR("Path request for index %d failed, this should not happen", i);
  }
  paths_pub_.publish(path_marker_array);
}

/**
 *  @brief update the start point, it is a callback funciton
 *  @param  pose    the start setting in Rviz
 */
void CorePathVisualPlugin::_onStartUpdate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  start_ = Point2D(pose->pose.pose.position.x, pose->pose.pose.position.y);
  Q_EMIT valueChanged();

  // mark pose on the map
  if (ros::ok())
  {
    visualization_msgs::Marker arrow, info;
    arrow.header.frame_id = info.header.frame_id = pose->header.frame_id;
    arrow.ns = "navigation_start_arrow";
    info.ns = "navigation_start_info";
    arrow.action = info.action = visualization_msgs::Marker::ADD;
    arrow.type = visualization_msgs::Marker::ARROW;
    info.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    arrow.pose = info.pose = pose->pose.pose;
    info.pose.position.z += 1.0;
    arrow.scale.x = 1.0;
    arrow.scale.y = 0.2;
    info.scale.z = 0.6;
    arrow.color.r = info.color.r = 0.0f;
    arrow.color.g = info.color.g = 0.0f;
    arrow.color.b = info.color.b = 1.0f;
    arrow.color.a = info.color.a = 0.7f;
    info.text = std::string("Start");
    marker_pub_.publish(arrow);
    marker_pub_.publish(info);
  }
  else
    ROS_ERROR("ROS node has been closed.");
}

/**
 *  @brief update the goal point, it is a callback funciton
 *  @param  pose    the goal user set in Rviz
 */
void CorePathVisualPlugin::_onGoalUpdate(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  goal_.x = pose->pose.position.x;
  goal_.y = pose->pose.position.y;
  Q_EMIT valueChanged();

  // mark pose on the map
  if (ros::ok())
  {
    visualization_msgs::Marker arrow, info;
    arrow.header.frame_id = info.header.frame_id = pose->header.frame_id;
    arrow.ns = "navigation_goal_arrow";
    info.ns = "navigation_goal_info";
    arrow.action = info.action = visualization_msgs::Marker::ADD;
    arrow.type = visualization_msgs::Marker::ARROW;
    info.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    arrow.pose = info.pose = pose->pose;
    info.pose.position.z += 1.0;
    arrow.scale.x = 1.0;
    arrow.scale.y = 0.2;
    info.scale.z = 0.6;
    arrow.color.r = info.color.r = 0.0f;
    arrow.color.g = info.color.g = 0.0f;
    arrow.color.b = info.color.b = 1.0f;
    arrow.color.a = info.color.a = 0.7f;
    info.text = std::string("Goal");
    marker_pub_.publish(arrow);
    marker_pub_.publish(info);
  }
  else
    ROS_ERROR("ROS node has been closed.");
}
}  // namespace path_visual_plugin
