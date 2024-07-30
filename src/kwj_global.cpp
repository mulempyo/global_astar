#include "kwj_global_planner/kwj_global.h"


//cost of non connected nodes
float infinity = std::numeric_limits<float>::infinity();

namespace kwj
{

  Kwj::Kwj(costmap_2d::Costmap2D* costmap)
  {
  ROS_WARN("construct Kwj");

  costmap_ = costmap;
  width = costmap_->getSizeInCellsX();
  height = costmap_->getSizeInCellsY();
  mapSize = width * height;
  resolution = costmap_->getResolution();

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

  ROS_WARN("construct Kwj2");
  }


bool Kwj::worldToMap(double wx, double wy, float& mx, float& my) const
{

  mx = (wx - costmap_->getOriginX()) / resolution;
  my = (wy - costmap_->getOriginY()) / resolution;

  if ((wx-costmap_->getOriginX()) > width*resolution || (wy-costmap_->getOriginY()) > height*resolution)
    {
     ROS_WARN("map false");
     return false;
    }
   else{
     ROS_WARN("map true");
    return true;}
}


vector<int> Kwj::runAStarOnGrid(int startGridSquare, int goalGridSquare)
{

  vector<int> bestPath;

  float g_score[mapSize];

  for (uint i = 0; i < mapSize; i++)
    g_score[i] = infinity;

  bestPath = findPath(startGridSquare, goalGridSquare, g_score);

  if (mapSize <= 0) {
    ROS_WARN("map size error");
    }
  else{  
    return bestPath;
  }
  
}

/**

  Generates the path for the bot towards the goal

**/
vector<int> Kwj::findPath(int startGridSquare, int goalGridSquare, float g_score[])
{
  value++;
  vector<int> bestPath;
  vector<int> emptyPath;
  cell gridSq;

  multiset<cell> openSquaresList;
  int currentGridSquare;

  //calculate g_score and f_score of the start position
  g_score[startGridSquare] = 0;
  gridSq.currentCell = startGridSquare;
  gridSq.fCost = g_score[startGridSquare] + calculateHScore(startGridSquare, goalGridSquare);

  //add the start gridSquare to the open list
  openSquaresList.insert(gridSq);
  currentGridSquare = startGridSquare;

  //while the open list is not empty and till goal square is reached continue the search
  while (!openSquaresList.empty() && g_score[goalGridSquare] == infinity)
  {
    //choose the gridSquare that has the lowest cost fCost in the open set
    currentGridSquare = openSquaresList.begin()->currentCell;
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
void Kwj::addNeighborGridSquareToOpenList(multiset<cell> &openSquaresList, int neighborGridSquare, int goalGridSquare, float g_score[])
{
  cell gridSq;
  gridSq.currentCell = neighborGridSquare; //insert the neighborGridSquare
  gridSq.fCost = g_score[neighborGridSquare] + calculateHScore(neighborGridSquare, goalGridSquare);
  openSquaresList.insert(gridSq);
}

/**
  Helper function to find free neighbours of currentGridSquare 
**/

vector<int> Kwj::findFreeNeighborGridSquare(int gridSquare)
{
  ROS_WARN("find free neightbor");
  int rowIndex = getGridSquareRowIndex(gridSquare);
  int colIndex = getGridSquareColIndex(gridSquare);
  int neighborIndex;
  vector<int> freeNeighborGridSquares;

  for (int i = -1; i <= 1; i++){
    for (int j = -1; j <= 1; j++)
    {
      //ROS_WARN("check whether the index is valid");
      //check whether the index is valid
      if ((rowIndex + i >= 0) && (rowIndex + i < height) && (colIndex + j >= 0) && (colIndex + j < width) && (!(i == 0 && j == 0)))
      {
        neighborIndex = ((rowIndex + i) * width) + (colIndex + j);

        if (isFree(neighborIndex))
         { freeNeighborGridSquares.push_back(neighborIndex);}
      }
    }
   }
  return freeNeighborGridSquares;
}

/**
  Checks if start and goal positions are valid and not unreachable.
**/
bool Kwj::isStartAndGoalValid(int startGridSquare, int goalGridSquare)
{
  ROS_WARN("start calc isStartAndGoalValid");
  bool isvalid = true;
  bool isFreeStartGridSquare = isFree(startGridSquare);

  if(isFreeStartGridSquare == true){ROS_WARN("startGridSquare true");}
  else{ROS_WARN("startGridSquare false");}

  bool isFreeGoalGridSquare = isFree(goalGridSquare);

  if(isFreeGoalGridSquare == true){ROS_WARN("goalGridSquare true");}
  else{ROS_WARN("goalGridSquare false");}

  ROS_WARN("free goalGridSquare isStartAndGoalValid");
  if (startGridSquare == goalGridSquare)
  {
    ROS_WARN("check whether the index is valid");
    isvalid = false;
  }
  else
  {
    if (!isFreeStartGridSquare && !isFreeGoalGridSquare)
    {
      ROS_WARN("false invalid1");
      isvalid = false;
    }
    else
    {
      if (!isFreeStartGridSquare)
      {
        ROS_WARN("false invalid2");
        isvalid = false;
      }
      else
      {
        if (!isFreeGoalGridSquare)
        {
          ROS_WARN("false invalid3");
          isvalid = false;
        }
        else
        {
          if (findFreeNeighborGridSquare(goalGridSquare).size() == 0)
          {
            ROS_WARN("false invalid4");
            isvalid = false;
          }
          else
          {
            if (findFreeNeighborGridSquare(startGridSquare).size() == 0)
            {
              ROS_WARN("false invalid5");
              isvalid = false;
            }
          }
        }
      }
    }
  }
  ROS_WARN("different startGridSquare goalGridSquare");
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
int Kwj::calculateGridSquareIndex(float i, float j) 
{
  return i * width + j;
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

void Kwj::getGridSquareCoordinates(int index, float &x, float &y)
  {
     x = getGridSquareColIndex(index) * costmap_->getResolution();
     y = getGridSquareRowIndex(index) * costmap_->getResolution();
     x = x + costmap_->getOriginX();
     y = y + costmap_->getOriginY();
  }

/**

  Checks if gridSquare at (i,j) is free

**/
bool Kwj::isFree(int i, int j)
{
  int gridSquareIndex = (i * costmap_->getSizeInCellsX()) + j;

  return occupancyGridMap[gridSquareIndex];
}

/**

  Checks if gridSquare at index gridSquareIndex is free

**/
bool Kwj::isFree(int gridSquareIndex)
{
  return occupancyGridMap[gridSquareIndex];
}


 Kwj::~Kwj(){
  ROS_WARN("delete Kwj");
    if(occupancyGridMap)
     {delete[] occupancyGridMap;}
      occupancyGridMap = nullptr;

    if(costmap_)
     {delete[] costmap_;}
      costmap_ = nullptr;
  }

};
bool operator<(cell const &c1, cell const &c2) { return c1.fCost < c2.fCost; }
