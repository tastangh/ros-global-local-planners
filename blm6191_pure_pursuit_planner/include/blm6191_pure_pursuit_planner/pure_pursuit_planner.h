#ifndef BLM6191_PURE_PURSUIT_PLANNER_H_
#define BLM6191_PURE_PURSUIT_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tf2_ros/buffer.h>

namespace blm6191_pure_pursuit_planner {

class PurePursuitPlanner : public nav_core::BaseLocalPlanner {
public:
    PurePursuitPlanner();
    ~PurePursuitPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool isGoalReached();

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

private:
    tf2_ros::Buffer* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    ros::NodeHandle private_nh_;

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    unsigned int current_waypoint_index_;

    bool initialized_;
    bool goal_reached_;

    double lookahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double goal_tolerance_;
    double transform_tolerance_;

    bool findLookaheadPoint(geometry_msgs::PoseStamped& lookahead_point);
    double calculateCurvature(const geometry_msgs::PoseStamped& lookahead_point_robot_frame) const;
    double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const;
};
} // namespace blm6191_pure_pursuit_planner

#endif // BLM6191_PURE_PURSUIT_PLANNER_H_