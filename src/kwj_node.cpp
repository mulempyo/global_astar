#include <kwj_global_planner/kwj_ros.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>

namespace cm=costmap_2d;
namespace rm=geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;

namespace kwj {

class KwjWithCostmap : public KwjROS
{
public:
  KwjWithCostmap(string name, Costmap2DROS* cmap);

private:
  void poseCallback(const rm::PoseStamped::ConstPtr& goal);
  Costmap2DROS* cmap_;
  ros::Subscriber pose_sub_;
};


void KwjWithCostmap::poseCallback(const rm::PoseStamped::ConstPtr& goal) {
  geometry_msgs::PoseStamped global_pose;
  cmap_->getRobotPose(global_pose);
  vector<PoseStamped> path;
  makePlan(global_pose, *goal, path);
}


KwjWithCostmap::KwjWithCostmap(string name, Costmap2DROS* cmap) : 
  KwjROS(name, cmap)
{
  ros::NodeHandle private_nh("~");
  cmap_ = cmap;
  pose_sub_ = private_nh.subscribe<rm::PoseStamped>("goal", 1, &KwjWithCostmap::poseCallback, this);
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "kwj_global_plan");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  costmap_2d::Costmap2DROS lcr("costmap", buffer);

  kwj::KwjWithCostmap kwj("kwj_planner", &lcr);

  ros::spin();
  return 0;
}
