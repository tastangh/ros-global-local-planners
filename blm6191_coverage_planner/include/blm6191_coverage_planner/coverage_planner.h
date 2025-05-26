#ifndef COVERAGE_PLANNER_H_
#define COVERAGE_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h> 
#include <geometry_msgs/Point32.h> 
#include <nav_msgs/Path.h> 
#include <vector>
#include <string>
#include <tf/tf.h> 
#include <tf/transform_listener.h> 

#include <cmath> 
#include <algorithm> 
#include <limits> 

namespace blm6191_coverage_planner {

class CoveragePlanner : public nav_core::BaseGlobalPlanner {
public:
    CoveragePlanner();
    ~CoveragePlanner();

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

private:
    costmap_2d::Costmap2DROS* costmap_ros_;

    tf::TransformListener* tf_; 

    ros::NodeHandle private_nh_;
    ros::Subscriber polygon_sub_;
    ros::Publisher path_pub_;

    geometry_msgs::Polygon coverage_polygon_;
    bool polygon_received_;

    double robot_radius_;

    bool initialized_;

    void polygonCallback(const geometry_msgs::Polygon::ConstPtr& msg);

    bool getHorizontalLineSegmentIntersection(double line_y,
                                              const geometry_msgs::Point32& p1,
                                              const geometry_msgs::Point32& p2,
                                              double& intersect_x) const;

    geometry_msgs::PoseStamped createPoseStamped(double x, double y, double yaw, const std::string& frame_id) const;

};
} 

#endif