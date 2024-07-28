#include <kwj_global_planner/kwj_ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(kwj::KwjROS, nav_core::BaseGlobalPlanner)


namespace kwj {

  KwjROS::KwjROS() 
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {}

  KwjROS::KwjROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {
      //initialize the planner
      initialize(name, costmap_ros);
  }

  KwjROS::~KwjROS()
  {
    if(occupancyGridMap)
      delete[] occupancyGridMap;
  }
  void KwjROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if (!initialized_)
  {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~/" + name);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

    private_nh.param("default_tolerance", default_tolerance_, 0.0);

    width = costmap_->getSizeInCellsX();
    height = costmap_->getSizeInCellsY();
    resolution = costmap_->getResolution();
    mapSize = width * height;
    tBreak = 1 + 1 / (mapSize);
    value = 0;

    occupancyGridMap = new bool[mapSize];
    for (unsigned int iy = 0; iy < height; iy++)
    {
      for (unsigned int ix = 0; ix < width; ix++)
      {
        unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

        if (cost == 0)
          occupancyGridMap[iy * width + ix] = true;
        else
          occupancyGridMap[iy * width + ix] = false;
      }
    }

    ROS_INFO("AStar planner initialized.");
    initialized_ = true;
  }
  else
    ROS_WARN("This planner has already been initialized... doing nothing");
  }
  
  void KwjROS::worldToMap(double wx, double wy, double& mx, double& my) {
    mx = (int)((wx - costmap_->getSizeInCellsX()) / costmap_->getResolution());
    my = (int)((wy - costmap_->getSizeInCellsY()) / costmap_->getResolution());
  }

  bool KwjROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        makePlan(start, goal, default_tolerance_, plan);
      }
  
  bool KwjROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan){
    if (!initialized_)
  {
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

  plan.clear();

  if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
  {
    ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
              costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    return false;
  }
  
    float startX = start.pose.position.x;
    float startY = start.pose.position.y;
    float goalX = goal.pose.position.x;
    float goalY = goal.pose.position.y;

    int start_index = planner_->calculateGridSquareIndex(startX, startY);
    int goal_index = planner_->calculateGridSquareIndex(goalX, goalY);

  if (!planner_->isStartAndGoalValid(start_index, goal_index)) {
    ROS_WARN("Invalid start or goal");
    return 0;
  }

  unsigned int mx, my;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  clearRobotCell(start, mx, my);

  std::vector<int> bestPath = planner_->runAStarOnGrid(start_index, goal_index);

  int len = bestPath.size();
  if (len > 0) {
    for (int i = 0; i < len; ++i) {
      float x = 0;
      float y = 0;
      float previous_x = 0;
      float previous_y = 0;

      planner_->getGridSquareCoordinates(bestPath[i], x, y);
      int previous_index;
      
      if(i != 0){  
         previous_index = bestPath[i-1];
         previous_x = x;
         previous_y = y;   
      }
      else{ 
      previous_index = bestPath[i];
      }

      planner_->getGridSquareCoordinates(previous_index, previous_x, previous_y);
    }
    return len;
  } else {
    ROS_WARN("No path found");
    return 0;
  }   
  publishPlan(plan);
  }
  
  void KwjROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //create a message for the plan 
    nav_msgs::Path gui_path;

    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();
    gui_path.poses.resize(path.size());
    

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }

  void KwjROS::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
  }
};
