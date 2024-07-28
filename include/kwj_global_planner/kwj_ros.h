#ifndef KWJ_KWJ_ROS_H_
#define KWJ_KWJ_ROS_H_

#include <ros/ros.h>
#include <kwj_global_planner/kwj_global.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>

namespace kwj {

  class KwjROS: public nav_core::BaseGlobalPlanner {
    public:
      int width;
      int height;
      float resolution;
      int mapSize = 0;
      int value = 0;
      bool *occupancyGridMap = nullptr;
      float tBreak = 0;

      KwjROS();
      KwjROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

     
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      bool makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal,double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

      ~KwjROS();


    protected:
      costmap_2d::Costmap2D* costmap_;
      costmap_2d::Costmap2DROS *costmap_ros_;
      boost::shared_ptr<Kwj> planner_;
      ros::Publisher plan_pub_;
      bool initialized_,allow_unknown_;


    private:
      void worldToMap(double wx, double wy, double& mx, double& my);
      void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
      double default_tolerance_;
      boost::mutex mutex_;
      std::string global_frame_;
  };
};

#endif
