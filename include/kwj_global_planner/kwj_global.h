
#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <set>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


using namespace std;
using std::string;

#ifndef KWJ_
#define KWJ_

/**
 * @struct GridSquares
 * @brief A struct that represents a GridSquare and its fCost.
 */
struct GridSquare
{
  int currentGridSquare;
  float fCost;
};

namespace kwj
{

class Kwj : public nav_core::BaseGlobalPlanner
{
public:
  ros::NodeHandle ROSNodeHandle;
  float originX;
  float originY;
  float resolution;
  costmap_2d::Costmap2DROS *costmap_ros_;
  costmap_2d::Costmap2D *costmap_;
  bool initialized_;
  double default_tolerance_;
  int width;
  int height;
  bool *occupancyGridMap;
  int mapSize;
  float infinity;
  ros::Publisher plan_pub_;
  ros::Publisher cloud_pub_;

  Kwj();
  Kwj(ros::NodeHandle &);
  Kwj(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  ~Kwj();



  
  /** overriden methods from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);

  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,double tolerance,
                std::vector<geometry_msgs::PoseStamped> &plan);

  vector<int> runAStarOnGrid(int startGridSquare, int goalGridSquare);
  vector<int> findPath(int startGridSquare, int goalGridSquare, float g_score[]);
  vector<int> constructPath(int startGridSquare, int goalGridSquare, float g_score[]);
  void addNeighborGridSquareToOpenList(multiset<GridSquare> &OPL, int neighborGridSquare, int goalGridSquare, float g_score[]);
  vector<int> findFreeNeighborGridSquare(int gridSquareIndex);
  bool isStartAndGoalValid(int startGridSquare, int goalGridSquare);
  float getMoveCost(int gridSquareIndex1, int gridSquareIndex2);
  float getMoveCost(int i1, int j1, int i2, int j2);
  float calculateHScore(int gridSquareIndex, int goalGridSquare);
  int calculateGridSquareIndex(int i, int j);
  int getGridSquareRowIndex(int index);
  int getGridSquareColIndex(int index);
  bool isFree(int gridSquareIndex); 
  bool isFree(int i, int j);

 // bool isGoalReached(const geometry_msgs::PoseStamped &current_pose, const geometry_msgs::PoseStamped &goal_pose, double tolerance);
  bool worldToMap(float wx, float wy, float& mx, float& my) const;
  void mapToWorld(int index, float &x, float &y);
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
  
};
};
#endif
