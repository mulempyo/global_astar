#include "kwj_global_planner/kwj_global.h"
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(kwj::Kwj, nav_core::BaseGlobalPlanner)

//cost of non connected nodes



namespace kwj
{

//Default Constructor
Kwj::Kwj(): mapSize(0), occupancyGridMap(nullptr), initialized_(false), infinity(std::numeric_limits<float>::infinity()){}

Kwj::Kwj(ros::NodeHandle &nh): mapSize(0), occupancyGridMap(nullptr), initialized_(false), infinity(std::numeric_limits<float>::infinity())
{ 
  ROSNodeHandle = nh;
}

/**
  Constructor that initilizes costmap and other parameters
**/
Kwj::Kwj(std::string name, costmap_2d::Costmap2DROS *costmap_ros): mapSize(0), occupancyGridMap(nullptr), initialized_(false), infinity(std::numeric_limits<float>::infinity())
{ 
  initialize(name, costmap_ros);
}

/**
Implementation of method from BaseGlobalPlanner interface that
initializes the cost map and other parameters of the grid.

**/

void Kwj::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{

  if (!initialized_)
  {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~/" + name);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan",1);
    private_nh.param("default_tolerance", default_tolerance_, 0.0);

    originX = costmap_->getOriginX();
    originY = costmap_->getOriginY();

    width = costmap_->getSizeInCellsX();
    height = costmap_->getSizeInCellsY();
    resolution = costmap_->getResolution();
    mapSize = width * height;

   
    occupancyGridMap = new bool[mapSize];
    for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
    {
      for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
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

/**
  Implementation of method from BaseGlobalPlanner interface that calculates
  plan to reach the goal
**/
bool Kwj::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                            std::vector<geometry_msgs::PoseStamped> &plan)
{
  return makePlan(start,goal,default_tolerance_,plan);
   
}


bool Kwj::makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,double tolerance,
                std::vector<geometry_msgs::PoseStamped> &plan)
{
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

  // convert the start and goal positions

  float wx = start.pose.position.x;
  float wy = start.pose.position.y;
  float mx = 0;
  float my = 0;

  if(!worldToMap(wx,wy,mx,my))
  {
    ROS_WARN(" failed to convert start pose to map coordinates");
    return false;
  }

  int start_index = calculateGridSquareIndex(my, mx);
  
  wx = goal.pose.position.x;
  wy = goal.pose.position.y;
  mx = 0;
  my = 0;

  if(!worldToMap(wx,wy,mx,my))
  {
    ROS_WARN(" failed to convert start pose to map coordinates");
    return false;
  }

  int goal_index = calculateGridSquareIndex(my, mx);

  //Convert to map coordinates relative to costmap origin


  // call global planner

  if (isStartAndGoalValid(start_index, goal_index))
  {

    vector<int> bestPath;
    bestPath.clear();

    bestPath = runAStarOnGrid(start_index, goal_index);

    //if the global planner finds a path
    if (bestPath.size() > 0)
    {

      // convert the path

      for (int i = 0; i < bestPath.size(); i++)
      {

        float x = 0.0;
        float y = 0.0;

        float previous_x = 0.0;
        float previous_y = 0.0;

        int index = bestPath[i];
        int previous_index;
        mapToWorld(index, x, y);

        if (i != 0)
        {
          previous_index = bestPath[i - 1];
        }
        else
        {
          previous_index = index;
        }

        mapToWorld(previous_index, previous_x, previous_y);

        //Orient the bot towards target
        tf::Vector3 vectorToTarget;
        vectorToTarget.setValue(x - previous_x, y - previous_y, 0.0);
        float angle = atan2((double)vectorToTarget.y(), (double)vectorToTarget.x());

        geometry_msgs::PoseStamped pose = goal;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

        plan.push_back(pose);
      }
      publishPlan(plan);
      return true;
    }

    else
    {
      ROS_WARN("The planner failed to find a path, choose other goal position");
      return false;
    }
  }

  else
  {
    ROS_WARN("Not valid start or goal");
    return false;
  }
}


/**
  Function to get gridSquare coordinates given index
  
**/
void Kwj::mapToWorld(int index, float &x, float &y)
{

  x = getGridSquareColIndex(index) * resolution + costmap_->getOriginX();

  y = getGridSquareRowIndex(index) * resolution + costmap_->getOriginY();
}

/**
  Function runs the A* algorithm to find best path to goal on grid

**/
vector<int> Kwj::runAStarOnGrid(int startGridSquare, int goalGridSquare)
{

  vector<int> bestPath;

  float g_score[mapSize];

  for (uint i = 0; i < mapSize; i++)
    g_score[i] = infinity;

  bestPath = findPath(startGridSquare, goalGridSquare, g_score);

  return bestPath;
}

/**

  Generates the path for the bot towards the goal

**/
vector<int> Kwj::findPath(int startGridSquare, int goalGridSquare, float g_score[])
{
  vector<int> bestPath;
  vector<int> emptyPath;
  GridSquare gridSq;

  multiset<GridSquare> openSquaresList;
  int currentGridSquare;

  //calculate g_score and f_score of the start position
  g_score[startGridSquare] = 0;
  gridSq.currentGridSquare = startGridSquare;
  gridSq.fCost = g_score[startGridSquare] + calculateHScore(startGridSquare, goalGridSquare);

  //add the start gridSquare to the open list
  openSquaresList.insert(gridSq);
  currentGridSquare = startGridSquare;

  //while the open list is not empty and till goal square is reached continue the search
  while (!openSquaresList.empty() && g_score[goalGridSquare] == infinity)
  {
    //choose the gridSquare that has the lowest cost fCost in the open set
    currentGridSquare = openSquaresList.begin()->currentGridSquare;
    //remove that gridSquare from the openList
    openSquaresList.erase(openSquaresList.begin());
    //search the neighbors of that gridSquare
    vector<int> neighborGridSquares;
    neighborGridSquares = findFreeNeighborGridSquare(currentGridSquare);
    for (uint i = 0; i < neighborGridSquares.size(); i++) //for each neighbor v of gridSquare
    {
      // if the g_score of the neighbor is equal to INF: unvisited gridSquare
      if (g_score[neighborGridSquares[i]] == infinity)
      {
        g_score[neighborGridSquares[i]] = g_score[currentGridSquare] + getMoveCost(currentGridSquare, neighborGridSquares[i]);
        addNeighborGridSquareToOpenList(openSquaresList, neighborGridSquares[i], goalGridSquare, g_score);
      }
    }
  }

  if (g_score[goalGridSquare] != infinity) // if goal gridSquare has been reached
  {
    bestPath = constructPath(startGridSquare, goalGridSquare, g_score);
    return bestPath;
  }
  else
  {
    ROS_INFO("Failure to find a path !");
    return emptyPath;
  }
}

/**
  Function constructs the path found by findPath function by returning vector of
  gridSquare indices that lie on path.

**/
vector<int> Kwj::constructPath(int startGridSquare, int goalGridSquare, float g_score[])
{
  vector<int> bestPath;
  vector<int> path;

  path.insert(path.begin() + bestPath.size(), goalGridSquare);
  int currentGridSquare = goalGridSquare;

  while (currentGridSquare != startGridSquare)
  {
    vector<int> neighborGridSquares;
    neighborGridSquares = findFreeNeighborGridSquare(currentGridSquare);

    vector<float> gScoresNeighbors;
    for (uint i = 0; i < neighborGridSquares.size(); i++)
      gScoresNeighbors.push_back(g_score[neighborGridSquares[i]]);

    int posMinGScore = distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
    currentGridSquare = neighborGridSquares[posMinGScore];

    //insert the neighbor in the path
    path.insert(path.begin() + path.size(), currentGridSquare);
  }
  for (uint i = 0; i < path.size(); i++)
    bestPath.insert(bestPath.begin() + bestPath.size(), path[path.size() - (i + 1)]);

  return bestPath;
}

/**
  Helper function to add unexplored neighbours of currentGridSquare to openlist
**/
void Kwj::addNeighborGridSquareToOpenList(multiset<GridSquare> &openSquaresList, int neighborGridSquare, int goalGridSquare, float g_score[])
{
  GridSquare gridSq;
  gridSq.currentGridSquare = neighborGridSquare; //insert the neighborGridSquare
  gridSq.fCost = g_score[neighborGridSquare] + calculateHScore(neighborGridSquare, goalGridSquare);
  openSquaresList.insert(gridSq);
}

/**
  Helper function to find free neighbours of currentGridSquare 
**/

vector<int> Kwj::findFreeNeighborGridSquare(int gridSquare)
{

  int rowIndex = getGridSquareRowIndex(gridSquare);
  int colIndex = getGridSquareColIndex(gridSquare);
  int neighborIndex;
  vector<int> freeNeighborGridSquares;

  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++)
    {
      //check whether the index is valid
      if ((rowIndex + i >= 0) && (rowIndex + i < height) && (colIndex + j >= 0) && (colIndex + j < width) && (!(i == 0 && j == 0)))
      {
        neighborIndex = ((rowIndex + i) * width) + (colIndex + j);

        if (isFree(neighborIndex))
          freeNeighborGridSquares.push_back(neighborIndex);
      }
    }
  return freeNeighborGridSquares;
}

/**
  Checks if start and goal positions are valid and not unreachable.
**/
bool Kwj::isStartAndGoalValid(int startGridSquare, int goalGridSquare)
{
  bool isvalid = true;
  bool isFreeStartGridSquare = isFree(startGridSquare);
  bool isFreeGoalGridSquare = isFree(goalGridSquare);
  if (startGridSquare == goalGridSquare)
  {

    isvalid = false;
  }
  else
  {
    if (!isFreeStartGridSquare && !isFreeGoalGridSquare)
    {

      isvalid = false;
    }
    else
    {
      if (!isFreeStartGridSquare)
      {

        isvalid = false;
      }
      else
      {
        if (!isFreeGoalGridSquare)
        {

          isvalid = false;
        }
        else
        {
          if (findFreeNeighborGridSquare(goalGridSquare).size() == 0)
          {

            isvalid = false;
          }
          else
          {
            if (findFreeNeighborGridSquare(startGridSquare).size() == 0)
            {

              isvalid = false;
            }
          }
        }
      }
    }
  }
  return isvalid;
}

/**
  Helper function to calculate cost of moving from currentGridSquare to neighbour

**/
float Kwj::getMoveCost(int i1, int j1, int i2, int j2)
{
  float moveCost = infinity; //start cost with maximum value. Change it to real cost of gridSquares are connected
  //if gridSquare(i2,j2) exists in the diagonal of gridSquare(i1,j1)
  if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1) || (i2 == i1 - 1 && j2 == j1 - 1) || (j2 == j1 - 1 && i2 == i1 + 1))
  {

    moveCost = 1.4;
  }
  //if gridSquare(i2,j2) exists in the horizontal or vertical line with gridSquare(i1,j1)
  else
  {
    if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1) || (i2 == i1 + 1 && j2 == j1) || (i1 == i2 && j2 == j1 + 1))
    {

      moveCost = 1;
    }
  }
  return moveCost;
}

/**
  Wrapper function to calculate cost of moving from currentGridSquare to neighbour

**/
float Kwj::getMoveCost(int gridSquareIndex1, int gridSquareIndex2)
{
  int i1 = 0, i2 = 0, j1 = 0, j2 = 0;

  i1 = getGridSquareRowIndex(gridSquareIndex1);
  j1 = getGridSquareColIndex(gridSquareIndex1);
  i2 = getGridSquareRowIndex(gridSquareIndex2);
  j2 = getGridSquareColIndex(gridSquareIndex2);

  return getMoveCost(i1, j1, i2, j2);
}

/**

**/
float Kwj::calculateHScore(int gridSquareIndex, int goalGridSquare)
{
  int x1 = getGridSquareRowIndex(goalGridSquare);
  int y1 = getGridSquareColIndex(goalGridSquare);
  int x2 = getGridSquareRowIndex(gridSquareIndex);
  int y2 = getGridSquareColIndex(gridSquareIndex);
  return abs(x1 - x2) + abs(y1 - y2);
}

/**
  Calculates the gridSquare index from square coordinates
**/
int Kwj::calculateGridSquareIndex(int i, int j) 
{
  return (i * width) + j;
}

/**

  Calculates gridSquare row from square index

**/
int Kwj::getGridSquareRowIndex(int index) //get the row index from gridSquare index
{
  return index / width;
}

/**

  Calculates gridSquare column from square index

**/
int Kwj::getGridSquareColIndex(int index) //get column index from gridSquare index
{
  return index % width;
}

/**

  Checks if gridSquare at (i,j) is free

**/
bool Kwj::isFree(int i, int j)
{
  int gridSquareIndex = (i * width) + j;

  return occupancyGridMap[gridSquareIndex];
}

/**

  Checks if gridSquare at index gridSquareIndex is free

**/
bool Kwj::isFree(int gridSquareIndex)
{
  return occupancyGridMap[gridSquareIndex];
}

 void Kwj::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //create a message for the plan 
    nav_msgs::Path gui_path;

    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;
    gui_path.poses.resize(path.size());
    

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }

bool Kwj::worldToMap(float wx, float wy, float& mx, float& my) const
{ 
 // ROS_WARN("worldToMap start");

  if((wx - costmap_->getOriginX()) > (width * resolution) || (wy - costmap_->getOriginY()) > (height * resolution))
     {return false;}
  else{
    mx = (wx - costmap_->getOriginX()) / resolution;
    my = (wy - costmap_->getOriginY()) / resolution;
   // ROS_WARN("worldToMap finish");
   return true;
    }

}
/*
void Kwj::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
    //ROS_WARN("clear_robot_cell");
  }
*/
Kwj::~Kwj()
{
  if(occupancyGridMap != nullptr)
   {delete[] occupancyGridMap;}
}

};

/**
  Overloaded operator for comparing cost among two gridSquares.

**/
bool operator<(GridSquare const &c1, GridSquare const &c2) { return c1.fCost < c2.fCost; }
