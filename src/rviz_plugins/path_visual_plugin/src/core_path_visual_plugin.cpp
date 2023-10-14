/***********************************************************
 *
 * @file: core_path_visual_plugin.cpp
 * @breif: Contains core of path visualization Rviz plugin class
 * @author: Yang Haodong, Wu Maojia
 * @update: 2023-10-14
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "wrapper_planner/CallPlan.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include "include/core_path_visual_plugin.h"

namespace path_visual_plugin
{
/**
   * @brief Construct a new CorePathVisualPlugin object
 */
CorePathVisualPlugin::CorePathVisualPlugin() : path_list_(new PathList)
{
  start_x_ = start_y_ = goal_x_ = goal_y_ = 0.00;
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
  plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

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
 * @brief Publish planning path
 * @param path  planning path
 */
void CorePathVisualPlugin::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  // create visulized path plan
  nav_msgs::Path gui_plan;
  gui_plan.poses.resize(plan.size());
  gui_plan.header.frame_id = "map";
  gui_plan.header.stamp = ros::Time::now();
  for (unsigned int i = 0; i < plan.size(); i++)
    gui_plan.poses[i] = plan[i];

  // publish plan to rviz
  plan_pub_.publish(gui_plan);
}

/**
 *  @brief call path planning service
 */
void CorePathVisualPlugin::addPath(const std::string& planner_name)
{
  geometry_msgs::PoseStamped start, goal;
  start.header.frame_id = "map";
  start.header.stamp = ros::Time::now();
  start.pose.position.x = start_x_;
  start.pose.position.y = start_y_;
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = goal_x_;
  goal.pose.position.y = goal_y_;

  wrapper_planner::CallPlan call_plan_srv;
  call_plan_srv.request.start = start;
  call_plan_srv.request.goal = goal;
  call_plan_srv.request.planner_name = planner_name;

  if (call_plan_client_.call(call_plan_srv))
  {
    publishPlan(call_plan_srv.response.path);

    if (path_list_->append(PathInfo{
      planner_name: planner_name,
      start_x: start_x_,
      start_y: start_y_,
      goal_x: goal_x_,
      goal_y: goal_y_,
      length: 0,
      turning_angle: 0,
      color: Qt::darkBlue,
      show: true
    }))
      ROS_INFO("Planner %s planning successfully done.", planner_name.c_str());
    else
    {
      ROS_ERROR("Planner %s planning failed.", planner_name.c_str());
      return;
    }
  }
  else
    ROS_ERROR("Planner %s planning failed.", planner_name.c_str());
}

/**
 *  @brief call load paths service
 */
void CorePathVisualPlugin::loadPaths()
{

}

/**
 *  @brief call save paths service
 */
void CorePathVisualPlugin::savePaths()
{

}

/**
 *  @brief set the color of path with some index
 *  @param index  the index of the path to set color
 *  @param color  the color to set
 */
void CorePathVisualPlugin::setPathColor(const int &index, const QColor& color)
{
    if (path_list_->setColor(index, color))
    {
      QRgb color_rgb = color.rgb();
      ROS_INFO("The color of path with index %d is successfully set to RGB(%d, %d, %d)!",
             index, qRed(color_rgb), qGreen(color_rgb), qBlue(color_rgb));


    }
    else
      ROS_ERROR("Failed to set the color of path with index %d.", index);
}

/**
 *  @brief set the show status of path with some index
 *  @param index  the index of the path to set show status
 *  @param show   whether to show the path or not
 */
void CorePathVisualPlugin::setPathShowStatus(const int& index, const bool& show)
{
    if (path_list_->setShow(index, show))
    {
      ROS_INFO("The show status of path with index %d is successfully set to %s!", index, show?"true":"false");


    }
    else
      ROS_ERROR("Failed to set the show status of path with index %d.", index);
}

/**
 *  @brief remove the path with some index from table view
 *  @param index  the index of the path to remove
 */
void CorePathVisualPlugin::removePath(const int &index)
{
  if (path_list_->remove(index))
  {
    ROS_INFO("Path with index %d is successfully removed!", index);


  }
  else
    ROS_ERROR("Failed to remove path with index %d.", index);
}

/**
 *  @brief update the start point, it is a callback funciton
 *  @param  pose    the start setting in Rviz
 */
void CorePathVisualPlugin::_onStartUpdate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  start_x_ = pose->pose.pose.position.x;
  start_y_ = pose->pose.pose.position.y;
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
  goal_x_ = pose->pose.position.x;
  goal_y_ = pose->pose.position.y;
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
