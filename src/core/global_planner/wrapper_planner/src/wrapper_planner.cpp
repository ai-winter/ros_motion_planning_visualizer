/***********************************************************
 *
 * @file: wrapper_planner.cpp
 * @breif: Contains the planner ROS wrapper class
 * @author: Yang Haodong
 * @update: 2023-10-2
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <pluginlib/class_list_macros.h>

#include "wrapper_planner.h"

// graph planner
#include "a_star.h"
#include "jump_point_search.h"
#include "d_star.h"
#include "lpa_star.h"
#include "d_star_lite.h"
#include "voronoi.h"
#include "theta_star.h"
#include "lazy_theta_star.h"

// sample planner
#include "rrt.h"
#include "rrt_star.h"
#include "rrt_connect.h"
#include "informed_rrt.h"

PLUGINLIB_EXPORT_CLASS(wrapper_planner::WrapperPlanner, nav_core::BaseGlobalPlanner)

namespace wrapper_planner
{
/**
 * @brief Construct a new Wrapper Planner object
 */
WrapperPlanner::WrapperPlanner() : initialized_(false), costmap_(nullptr), g_planner_(nullptr)
{
}

/**
 * @brief Construct a new Wrapper Planner object
 * @param name        planner name
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
WrapperPlanner::WrapperPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : WrapperPlanner()
{
  initialize(name, costmap_ros);
}

/**
 * @brief Destroy the Wrapper Planner object
 */
WrapperPlanner::~WrapperPlanner()
{
  if (g_planner_)
  {
    delete g_planner_;
    g_planner_ = NULL;
  }
}

/**
 * @brief Planner initialization
 * @param name       planner name
 * @param costmapRos costmap ROS wrapper
 */
void WrapperPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos)
{
  costmap_ros_ = costmapRos;
  initialize(name);
}

/**
 * @brief Planner initialization
 * @param name     planner name
 * @param costmap  costmap pointer
 * @param frame_id costmap frame ID
 */
void WrapperPlanner::initialize(std::string name)
{
  if (!initialized_)
  {
    initialized_ = true;

    // initialize ROS node
    ros::NodeHandle private_nh("~/" + name);

    // initialize costmap
    costmap_ = costmap_ros_->getCostmap();

    // costmap frame ID
    frame_id_ = costmap_ros_->getGlobalFrameID();

    // get costmap properties
    nx_ = costmap_->getSizeInCellsX(), ny_ = costmap_->getSizeInCellsY();
    origin_x_ = costmap_->getOriginX(), origin_y_ = costmap_->getOriginY();
    resolution_ = costmap_->getResolution();

    private_nh.param("convert_offset", convert_offset_, 0.0);  // offset of transform from world(x,y) to grid
    private_nh.param("default_tolerance", tolerance_, 0.0);    // error tolerance
    private_nh.param("outline_map", is_outline_, false);       // whether outline the map or not

    call_plan_srv_ = private_nh.advertiseService("call_plan", &WrapperPlanner::callPlanService, this);

    ROS_INFO("Prepare for path visualization.");
  }
  else
    ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

/**
 * @brief plan a path given start and goal in world map
 * @param start start in world map
 * @param goal  goal in world map
 * @param plan  plan
 * @return true if find a path successfully, else false
 */
bool WrapperPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan)
{
  return makePlan(start, goal, tolerance_, plan);
}

/**
 * @brief Plan a path given start and goal in world map
 * @param start     start in world map
 * @param goal      goal in world map
 * @param plan      plan
 * @param tolerance error tolerance
 * @return true if find a path successfully, else false
 */
bool WrapperPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                              double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{
}

/**
 * @brief Call planner to plan the path
 * @param req  service request, including start, goal and planner name
 * @param resp service response, including planning path and information
 * @return true if find a path successfully, else false
 */
bool WrapperPlanner::callPlanService(CallPlan::Request& req, CallPlan::Response& resp)
{
  // build planner
  if (req.planner_name == "a_star")
    g_planner_ = new global_planner::AStar(nx_, ny_, resolution_);
  else if (req.planner_name == "dijkstra")
    g_planner_ = new global_planner::AStar(nx_, ny_, resolution_, true);
  else if (req.planner_name == "gbfs")
    g_planner_ = new global_planner::AStar(nx_, ny_, resolution_, false, true);
  else if (req.planner_name == "jps")
    g_planner_ = new global_planner::JumpPointSearch(nx_, ny_, resolution_);
  else if (req.planner_name == "d_star")
    g_planner_ = new global_planner::DStar(nx_, ny_, resolution_);
  else if (req.planner_name == "lpa_star")
    g_planner_ = new global_planner::LPAStar(nx_, ny_, resolution_);
  else if (req.planner_name == "d_star_lite")
    g_planner_ = new global_planner::DStarLite(nx_, ny_, resolution_);
  else if (req.planner_name == "voronoi")
    g_planner_ = new global_planner::VoronoiPlanner(nx_, ny_, resolution_,
                                                    costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
  else if (req.planner_name == "theta_star")
    g_planner_ = new global_planner::ThetaStar(nx_, ny_, resolution_);
  else if (req.planner_name == "lazy_theta_star")
    g_planner_ = new global_planner::LazyThetaStar(nx_, ny_, resolution_);
  else if (req.planner_name == "rrt")
    g_planner_ = new global_planner::RRT(nx_, ny_, resolution_, 2000, 10.0);
  else if (req.planner_name == "rrt_star")
    g_planner_ = new global_planner::RRTStar(nx_, ny_, resolution_, 2000, 10.0, 20.0);
  else if (req.planner_name == "rrt_connect")
    g_planner_ = new global_planner::RRTConnect(nx_, ny_, resolution_, 2000, 10.0);
  else if (req.planner_name == "informed_rrt")
    g_planner_ = new global_planner::InformedRRT(nx_, ny_, resolution_, 3000, 10.0, 20.0);
  else
    ROS_ERROR("Unknown planner name: %s", req.planner_name.c_str());

  ROS_INFO("Using global global planner: %s", req.planner_name.c_str());

  // get goal and strat node coordinate tranform from world to costmap
  double wx = req.start.pose.position.x, wy = req.start.pose.position.y;
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if (!_worldToMap(wx, wy, m_start_x, m_start_y))
  {
    ROS_WARN(
        "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has "
        "been properly localized?");
    return false;
  }
  wx = req.goal.pose.position.x, wy = req.goal.pose.position.y;
  if (!_worldToMap(wx, wy, m_goal_x, m_goal_y))
  {
    ROS_WARN_THROTTLE(1.0,
                      "The goal sent to the global planner is off the global costmap. Planning will always fail to "
                      "this goal.");
    return false;
  }

  // tranform from costmap to grid map
  int g_start_x, g_start_y, g_goal_x, g_goal_y;
  g_planner_->map2Grid(m_start_x, m_start_y, g_start_x, g_start_y);
  g_planner_->map2Grid(m_goal_x, m_goal_y, g_goal_x, g_goal_y);

  // NOTE: how to init start and goal?
  global_planner::Node start_node(g_start_x, g_start_y, 0, 0, g_planner_->grid2Index(g_start_x, g_start_y), 0);
  global_planner::Node goal_node(g_goal_x, g_goal_y, 0, 0, g_planner_->grid2Index(g_goal_x, g_goal_y), 0);

  // outline the map
  if (is_outline_)
    g_planner_->outlineMap(costmap_->getCharMap());

  // calculate path
  std::vector<global_planner::Node> path;
  std::vector<global_planner::Node> expand;
  bool path_found = false;
  ros::Publisher plan_pub_;  // path planning publisher
  if (req.planner_name == "voronoi")
  {
    bool voronoi_layer_exist = false;
    // check if the costmap has a Voronoi layer
    for (auto layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
         layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
    {
      boost::shared_ptr<costmap_2d::VoronoiLayer> voronoi_layer =
          boost::dynamic_pointer_cast<costmap_2d::VoronoiLayer>(*layer);
      if (voronoi_layer)
      {
        voronoi_layer_exist = true;
        global_planner::VoronoiData** voronoi_diagram;
        voronoi_diagram = new global_planner::VoronoiData*[nx_];
        for (unsigned int i = 0; i < nx_; i++)
          voronoi_diagram[i] = new global_planner::VoronoiData[ny_];

        boost::unique_lock<boost::mutex> lock(voronoi_layer->getMutex());
        const DynamicVoronoi& voronoi = voronoi_layer->getVoronoi();
        for (unsigned int j = 0; j < ny_; j++)
        {
          for (unsigned int i = 0; i < nx_; i++)
          {
            voronoi_diagram[i][j].dist = voronoi.getDistance(i, j) * resolution_;
            voronoi_diagram[i][j].is_voronoi = voronoi.isVoronoi(i, j);
          }
        }
        path_found = dynamic_cast<global_planner::VoronoiPlanner*>(g_planner_)
                         ->plan(voronoi_diagram, start_node, goal_node, path);
        break;
      }
    }
    if (!voronoi_layer_exist)
      ROS_ERROR("Failed to get a Voronoi layer for Voronoi planner");
  }
  else
    path_found = g_planner_->plan(costmap_->getCharMap(), start_node, goal_node, path, expand);

  std::vector<geometry_msgs::PoseStamped> plan;
  if (path_found)
  {
    if (_getPlanFromPath(path, plan))
    {
      geometry_msgs::PoseStamped goalCopy = req.goal;
      goalCopy.header.stamp = ros::Time::now();
      plan.push_back(goalCopy);
      resp.path = plan;
    }
    else
      ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
  }

  resp.plan_found = path_found;

  return !plan.empty();
}

/**
 * @brief Calculate plan from planning path
 * @param path path generated by global planner
 * @param plan plan transfromed from path, i.e. [start, ..., goal]
 * @return bool true if successful, else false
 */
bool WrapperPlanner::_getPlanFromPath(std::vector<global_planner::Node>& path,
                                      std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  std::string globalFrame = frame_id_;
  ros::Time planTime = ros::Time::now();
  plan.clear();

  for (int i = path.size() - 1; i >= 0; i--)
  {
    double wx, wy;
    _mapToWorld((double)path[i].x_, (double)path[i].y_, wx, wy);

    // coding as message type
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }

  return !plan.empty();
}

/**
 * @brief Tranform from costmap(x, y) to world map(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 */
void WrapperPlanner::_mapToWorld(double mx, double my, double& wx, double& wy)
{
  wx = origin_x_ + (mx + convert_offset_) * resolution_;
  wy = origin_y_ + (my + convert_offset_) * resolution_;
}

/**
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 * @return true if successfull, else false
 */
bool WrapperPlanner::_worldToMap(double wx, double wy, double& mx, double& my)
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (wx - origin_x_) / resolution_ - convert_offset_;
  my = (wy - origin_y_) / resolution_ - convert_offset_;
  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
    return true;

  return false;
}
}  // namespace wrapper_planner