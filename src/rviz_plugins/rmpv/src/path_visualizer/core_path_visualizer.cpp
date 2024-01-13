/***********************************************************
 *
 * @file: core_path_visualizer.cpp
 * @breif: Contains core of path visualizer class
 * @author: Wu Maojia, Yang Haodong
 * @update: 2024-1-12
 * @version: 1.0
 *
 * Copyright (c) 2024， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "path_visualizer/core_path_visualizer.h"

namespace rmpv
{
/**
 * @brief Construct a new CorePathVisualizer object
 */
CorePathVisualizer::CorePathVisualizer() : path_list_(new PathList)
{
  start_ = goal_ = Pose2D(0.0, 0.0, 0.0);
}

/**
 * @brief Destroy the CorePathVisualizer object
 */
CorePathVisualizer::~CorePathVisualizer()
{
  if (path_list_)
    delete path_list_;
}

/**
 * @brief ROS parameters initialization
 */
void CorePathVisualizer::setupROS()
{
  // initialize ROS node
  ros::NodeHandle private_nh("");

  // publishers
  marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  paths_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("/paths", 10);
  start_pub_ = private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
  goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

  // subscribers
  start_sub_ = private_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1, boost::bind(&CorePathVisualizer::_onStartUpdate, this, _1));
  goal_sub_ = private_nh.subscribe<geometry_msgs::PoseStamped>(
      "move_base_simple/goal", 1, boost::bind(&CorePathVisualizer::_onGoalUpdate, this, _1));

  // client
  call_plan_client_ = private_nh.serviceClient<wrapper_planner::CallPlan>("/move_base/WrapperPlanner/call_plan");

  // parameters
  private_nh.getParam("/move_base/planner", planner_list_);

  // refresh poses and paths displayed in rviz
  refresh_poses();
  refresh_paths();
}

/**
 *  @brief call path planning service
 *  @param planner_name name of planner
 */
void CorePathVisualizer::addPath(const QString& planner_name)
{
  geometry_msgs::PoseStamped start, goal;
  start.header.frame_id = "map";
  start.header.stamp = ros::Time::now();
  start.pose.position.x = start_.x;
  start.pose.position.y = start_.y;
  start.pose.orientation = tf::createQuaternionMsgFromYaw(start_.yaw);
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = goal_.x;
  goal.pose.position.y = goal_.y;
  goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.yaw);

  wrapper_planner::CallPlan call_plan_srv;
  call_plan_srv.request.start = start;
  call_plan_srv.request.goal = goal;
  call_plan_srv.request.planner_name = planner_name.toStdString();

  ROS_WARN("call planner %s", planner_name.toStdString().c_str());
  ROS_WARN("start: %f, %f, %f", start.pose.position.x, start.pose.position.y, start_.yaw);
  ROS_WARN("goal: %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal_.yaw);

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
      refresh_paths();
      ROS_WARN("Planner %s planning successfully done.", planner_name.toStdString().c_str());
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
 *  @brief call save paths service
 *  @param save_file  save paths to local workspace using .json format
 */
void CorePathVisualizer::savePaths(const QString& save_file)
{
  ROS_WARN("Saving path information at location %s", save_file.toStdString().c_str());
  path_list_->save(save_file);
}

/**
 *  @brief call load paths service
 *  @param open_file  load paths from local workspace using .json format
 */
void CorePathVisualizer::loadPaths(const QString open_file)
{
  ROS_WARN("Loading path information at location %s", open_file.toStdString().c_str());
  path_list_->load(open_file);
  refresh_paths();
}

/**
 *  @brief set the color of path with some index
 *  @param index  the index of the path to set color
 *  @param color  the color to set
 */
void CorePathVisualizer::setPathColor(const int& index, const QColor& color)
{
  if (path_list_->setColor(index, color))
  {
    QRgb color_rgb = color.rgb();
    ROS_WARN("The color of path with index %d is successfully set to RGB(%d, %d, %d)!", index, qRed(color_rgb),
             qGreen(color_rgb), qBlue(color_rgb));
    refresh_paths();
  }
  else
    ROS_ERROR("Failed to set the color of path with index %d.", index);
}

/**
 *  @brief set the select status of path with some index
 *  @param index  the index of the path to set select status
 *  @param select   whether to select and visualize the path or not
 */
void CorePathVisualizer::setPathSelectStatus(const int& index, const bool& select)
{
  if (path_list_->setSelect(index, select))
  {
    ROS_WARN("The select status of path with index %d is successfully set to %s!", index, select ? "true" : "false");
    refresh_paths();
  }
  else
    ROS_ERROR("Failed to set the select status of path with index %d.", index);
}

/**
 *  @brief remove the path with some index from table view
 *  @param index  the index of the path to remove
 */
void CorePathVisualizer::removePath(const int& index)
{
  if (path_list_->remove(index))
  {
    ROS_WARN("Path with index %d is successfully removed!", index);
  }
  else
    ROS_ERROR("Failed to remove path with index %d.", index);
}

/**
 *  @brief refresh paths displayed in rviz
 */
void CorePathVisualizer::refresh_paths()
{
  visualization_msgs::Marker path_marker;
  visualization_msgs::MarkerArray path_marker_array;
  int marker_id = 0;

  for (const auto& info : *(path_list_->getListPtr()))
  {
    QList<Point2D> path_points = info.getPathPoints();
    for (unsigned int i = 0; i < path_points.size() - 1; i++)
    {
      path_marker.ns = "path_marker";
      path_marker.type = visualization_msgs::Marker::LINE_LIST;
      path_marker.action = path_marker.ADD;
      geometry_msgs::Point p;
      p.x = path_points[i].x;
      p.y = path_points[i].y;
      p.z = 0.0;
      path_marker.points.push_back(p);
      p.x = path_points[i + 1].x;
      p.y = path_points[i + 1].y;
      p.z = 0.0;
      path_marker.points.push_back(p);

      path_marker.pose.orientation.x = 0.0;
      path_marker.pose.orientation.y = 0.0;
      path_marker.pose.orientation.z = 0.0;
      path_marker.pose.orientation.w = 1.0;
      path_marker.scale.x = 0.1;

      // set path color
      QColor path_color = info.getData(PathInfo::pathColor).value<QColor>();
      path_marker.color.r = path_color.red() / 255.0;
      path_marker.color.g = path_color.green() / 255.0;
      path_marker.color.b = path_color.blue() / 255.0;
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
  paths_pub_.publish(path_marker_array);
}

/**
 *  @brief refresh start and goal poses displayed in rviz
 */
void CorePathVisualizer::refresh_poses() {
  // start pose topic "/initialpose"
  // goal pose topic "move_base_simple/goal"
  geometry_msgs::PoseWithCovarianceStamped start_pose;
  geometry_msgs::PoseStamped goal_pose;
  start_.normalizeYaw();
  goal_.normalizeYaw();
  Q_EMIT valueChanged();  // normalization may change the yaw value

  start_pose.header.frame_id = "map";
  start_pose.header.stamp = ros::Time::now();
  start_pose.pose.pose.position.x = start_.x;
  start_pose.pose.pose.position.y = start_.y;
  start_pose.pose.pose.position.z = 0.0;
  start_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(start_.yaw);

  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.pose.position.x = goal_.x;
  goal_pose.pose.position.y = goal_.y;
  goal_pose.pose.position.z = 0.0;
  goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.yaw);

  start_pub_.publish(start_pose);
  goal_pub_.publish(goal_pose);
}

/**
 *  @brief update the start pose, it is a callback funciton
 *  @param  pose    the start setting in Rviz
 */
void CorePathVisualizer::_onStartUpdate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  start_ = Pose2D(pose->pose.pose.position.x, pose->pose.pose.position.y, tf::getYaw(pose->pose.pose.orientation));
  Q_EMIT valueChanged();  // normalization may change the yaw value

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
 *  @brief update the goal pose, it is a callback funciton
 *  @param  pose    the goal user set in Rviz
 */
void CorePathVisualizer::_onGoalUpdate(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  goal_ = Pose2D(pose->pose.position.x, pose->pose.position.y, tf::getYaw(pose->pose.orientation));
  Q_EMIT valueChanged();  // normalization may change the yaw value

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
}  // namespace rmpv
