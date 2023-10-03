/***********************************************************
 *
 * @file: path_visual_plugin.cpp
 * @breif: Contains path visualization Rviz plugin class
 * @author: Yang Haodong
 * @update: 2023-10-2
 * @version: 1.1
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "wrapper_planner/CallPlan.h"
#include "include/path_visual_plugin.h"
#include "include/ui_path_visual_plugin.h"

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(path_visual_plugin::PathVisualPlugin, rviz::Panel)

namespace path_visual_plugin
{
/**
 * @brief Construct a new Path Visualization Plugin object
 */
PathVisualPlugin::PathVisualPlugin(QWidget* parent) : rviz::Panel(parent), ui(new Ui::PathVisualPlugin)
{
  ui->setupUi(this);
  setupROS();
  setupUi();
}

/**
 * @brief Destroy the Path Visualization Plugin object
 */
PathVisualPlugin::~PathVisualPlugin()
{
  delete ui;
}

/**
 * @brief User interface parameters initialization
 */
void PathVisualPlugin::setupUi()
{
  for (const auto p_name : planner_list_)
    ui->comboBox->addItem(QString::fromStdString(p_name));

  connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(_onPlanPath()));
}

/**
 * @brief ROS parameters initialization
 */
void PathVisualPlugin::setupROS()
{
  // initialize ROS node
  ros::NodeHandle private_nh("");

  // publisher
  marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

  // subscriber
  goal_sub_ = private_nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1,
                                                               boost::bind(&PathVisualPlugin::_onGoalUpdate, this, _1));
  start_sub_ = private_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1, boost::bind(&PathVisualPlugin::_onStartUpdate, this, _1));

  // client
  call_plan_client_ = private_nh.serviceClient<wrapper_planner::CallPlan>("/move_base/WrapperPlanner/call_plan");

  // parameters
  private_nh.getParam("/move_base/planner", planner_list_);
}

/**
 *  @brief update the goal point, it is a callback funciton
 *  @param  pose    the goal user set in Rviz
 */
void PathVisualPlugin::_onGoalUpdate(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  goal_x_ = pose->pose.position.x;
  goal_y_ = pose->pose.position.y;
  ui->lineEdit_3->setText(QString::number(goal_x_, 'f', 2));
  ui->lineEdit_4->setText(QString::number(goal_y_, 'f', 2));

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

/**
 *  @brief update the start point, it is a callback funciton
 *  @param  pose    the start setting in Rviz
 */
void PathVisualPlugin::_onStartUpdate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  start_x_ = pose->pose.pose.position.x;
  start_y_ = pose->pose.pose.position.y;
  ui->lineEdit->setText(QString::number(start_x_, 'f', 2));
  ui->lineEdit_2->setText(QString::number(start_y_, 'f', 2));

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
 *  @brief call path planning service, it is a callback funciton
 */
void PathVisualPlugin::_onPlanPath()
{
  std::string planner_name = ui->comboBox->currentText().toStdString();
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
    ROS_INFO("Planner %s planning successfully done.", planner_name.c_str());
  }
  else
    ROS_WARN("Planner %s planning failed.", planner_name.c_str());
}

/**
 * @brief Publish planning path
 * @param path  planning path
 */
void PathVisualPlugin::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
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
}  // namespace path_visual_plugin
