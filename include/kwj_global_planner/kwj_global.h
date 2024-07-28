#ifndef _KWJ_H_
#define _KWJ_H_

#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <set>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>


using namespace std;
using std::string;

struct cell
{
  int currentCell;
  float fCost;
};

namespace kwj
{

class Kwj
{
public:
  int mapSize = 0;
  int value = 0;
  bool *occupancyGridMap = nullptr;
  float tBreak = 0;
  ros::NodeHandle nh;
  float resolution;
  costmap_2d::Costmap2DROS *costmap_ros_;
  costmap_2d::Costmap2D *costmap_;
  bool initialized_;
  int width;
  int start[2];
  int goal[2];
 
  Kwj();
  ~Kwj();
  /** Helper methods**/
  void setGoal(int *g);
  void setStart(int *g);
  vector<int> runAStarOnGrid(int startGridSquare, int goalGridSquare);
  vector<int> findPath(int startGridSquare, int goalGridSquare, float g_score[]);
  vector<int> constructPath(int startGridSquare, int goalGridSquare, float g_score[]);
  void addNeighborGridSquareToOpenList(multiset<cell> &OPL, int neighborGridSquare, int goalGridSquare, float g_score[]);
  vector<int> findFreeNeighborGridSquare(int gridSquareIndex);
  bool isStartAndGoalValid(int startGridSquare, int goalGridSquare);
  float getMoveCost(int gridSquareIndex1, int gridSquareIndex2);
  float getMoveCost(int i1, int j1, int i2, int j2);
  float calculateHScore(int gridSquareIndex, int goalGridSquare);

  int calculateGridSquareIndex(float i, float j);
 
  int getGridSquareRowIndex(int index);
  
  int getGridSquareColIndex(int index);

  void getGridSquareCoordinates(int index, float &x, float &y);

  bool isFree(int gridSquareIndex); 

  bool isFree(int i, int j);
  
};
};
#endif
