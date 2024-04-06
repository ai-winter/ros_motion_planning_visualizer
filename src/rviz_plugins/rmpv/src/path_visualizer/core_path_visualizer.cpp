/**
 * *********************************************************
 *
 * @file: core_path_visualizer.cpp
 * @brief: Contains core of path visualizer class
 * @author: Wu Maojia, Yang Haodong
 * @date: 2024-4-6
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong, Wu Maojia. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
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

  // server
  im_server_ = new interactive_markers::InteractiveMarkerServer("rmpv_interactive_markers");
  im_server_->clear();
  im_server_->insert(makeInteractiveMarker("start", start_), boost::bind(&CorePathVisualizer::_onStartUpdate, this, _1));
  im_server_->insert(makeInteractiveMarker("goal", goal_), boost::bind(&CorePathVisualizer::_onGoalUpdate, this, _1));
  im_server_->applyChanges();

  // client
  call_plan_client_ = private_nh.serviceClient<wrapper_planner::CallPlan>("/move_base/WrapperPlanner/call_plan");

  // publishers
  paths_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("/rmpv_paths", 10);

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
  start.pose.orientation = tf::createQuaternionMsgFromYaw(start_.theta);
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = goal_.x;
  goal.pose.position.y = goal_.y;
  goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.theta);

  wrapper_planner::CallPlan call_plan_srv;
  call_plan_srv.request.start = start;
  call_plan_srv.request.goal = goal;
  call_plan_srv.request.planner_name = planner_name.toStdString();

  ROS_WARN("call planner %s", planner_name.toStdString().c_str());
  ROS_WARN("start: %f, %f, %f", start.pose.position.x, start.pose.position.y, start_.theta);
  ROS_WARN("goal: %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal_.theta);

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
    refresh_paths();
  }
  else
    ROS_ERROR("Failed to remove path with index %d.", index);
}

/**
 *  @brief refresh paths displayed in rviz
 */
void CorePathVisualizer::refresh_paths()
{
  visualization_msgs::MarkerArray path_marker_array;
  visualization_msgs::Marker path_marker;

  path_marker.action = path_marker.DELETEALL;
  path_marker_array.markers.push_back(path_marker);
  path_marker.points.clear();

  int marker_id = 0;

  for (const auto& info : *(path_list_->getListPtr()))
  {
    if (!info.getData(PathInfo::selectStatus).value<bool>())
      continue;

    QList<Point2D> path_points = info.getPathPoints();
    for (unsigned int i = 0; i < path_points.size() - 1; i++)
    {
      path_marker.ns = "path_marker";
      path_marker.type = visualization_msgs::Marker::LINE_STRIP;
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
  start_.normalizeTheta();
  goal_.normalizeTheta();
  Q_EMIT valueChanged();
  im_server_->insert(makeInteractiveMarker("start", start_), boost::bind(&CorePathVisualizer::_onStartUpdate, this, _1));
  im_server_->insert(makeInteractiveMarker("goal", goal_), boost::bind(&CorePathVisualizer::_onGoalUpdate, this, _1));
  im_server_->applyChanges();
}

/**
 *  @brief update the start pose marker, it is a callback funciton
 *  @param int_marker  interactive marker message
 */
void CorePathVisualizer::_onStartUpdate(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& int_marker)
{
  const geometry_msgs::Pose& pose = int_marker->pose;
  start_ = Pose2D(pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
  Q_EMIT valueChanged();  // normalization may change the theta value
}

/**
 *  @brief update the goal pose marker, it is a callback funciton
 *  @param int_marker  interactive marker message
 */
void CorePathVisualizer::_onGoalUpdate(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& int_marker)
{
  const geometry_msgs::Pose& pose = int_marker->pose;
  goal_ = Pose2D(pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
  Q_EMIT valueChanged();  // normalization may change the theta value
}

/**
 * @brief make an interactive marker for pose
 * @param name  name of the marker
 * @param pose  pose of the marker
 * @return an interactive marker message
 */
visualization_msgs::InteractiveMarker CorePathVisualizer::makeInteractiveMarker(const std::string& name, const Pose2D& pose)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.name = name;
//  int_marker.description = name;
  int_marker.pose.position.x = pose.x;
  int_marker.pose.position.y = pose.y;
  int_marker.pose.position.z = 0.01;
  int_marker.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);

  visualization_msgs::Marker arrow_marker;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.scale.x = 1.0;
  arrow_marker.scale.y = 0.1;
  arrow_marker.scale.z = 0.1;
  if (name == "start")
  {
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 1.0;
    arrow_marker.color.b = 0.0;
  }
  else if (name == "goal")
  {
    arrow_marker.color.r = 1.0;
    arrow_marker.color.g = 0.0;
    arrow_marker.color.b = 0.0;
  }
  else
  {
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 0.0;
    arrow_marker.color.b = 1.0;
  }
  arrow_marker.color.a = 1.0;

  visualization_msgs::Marker name_marker;
  name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  name_marker.text = name;
  name_marker.scale.z = 0.4;
  name_marker.color.r = arrow_marker.color.r;
  name_marker.color.g = arrow_marker.color.g;
  name_marker.color.b = arrow_marker.color.b;
  name_marker.color.a = arrow_marker.color.a;
  name_marker.pose.position.z = 0.2;

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);

  visualization_msgs::InteractiveMarkerControl control_arrow;
  control_arrow.always_visible = true;
  control_arrow.markers.push_back(arrow_marker);
  int_marker.controls.push_back(control_arrow);

  visualization_msgs::InteractiveMarkerControl control_name;
  control_name.always_visible = true;
  control_name.markers.push_back(name_marker);
  int_marker.controls.push_back(control_name);

  return int_marker;
}

}  // namespace rmpv
