#ifndef COVERAGE_PLANNER_H_
#define COVERAGE_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h> // As specified by assignment
#include <geometry_msgs/Point32.h> // Need this for polygon points
#include <nav_msgs/Path.h> // Need this for publishing the path
#include <vector>
#include <string>
#include <tf/tf.h> // For tf::createQuaternionMsgFromYaw (part of tf)
#include <tf/transform_listener.h> // Need this for tf::TransformListener
// tf/transform_datatypes.h ve tf/exceptions.h genellikle tf/tf.h ve tf/transform_listener.h tarafından çekilir
// ancak bazen explicit include gerekebilir. tf.h genellikle yeterli olur.
// #include <tf/transform_datatypes.h>
// #include <tf/exceptions.h>

#include <cmath> // For M_PI, std::abs, std::sin, std::cos, std::atan2, std::sqrt etc.
#include <algorithm> // For std::min, std::max, std::sort
#include <limits> // For std::numeric_limits

namespace blm6191_coverage_planner {

class CoveragePlanner : public nav_core::BaseGlobalPlanner {
public:
    CoveragePlanner();
    ~CoveragePlanner();

    /**
     * @brief Initializer function for the BaseGlobalPlanner
     * @param name The name of this planner
     * @param costmap_ros The costmap2DROS object associated with this planner
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start The start pose for the plan (used for frame reference and optional path start)
     * @param goal The goal pose for the plan (used here as a trigger, not necessarily path endpoint)
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

private:
    // Costmap pointer (optional for basic polygon coverage, useful for obstacle avoidance)
    costmap_2d::Costmap2DROS* costmap_ros_;
    // costmap_2d::Costmap2D* costmap_; // Pointer to the underlying costmap (not used in this simple impl)

    // We need our own TF listener instance to transform the start pose
    tf::TransformListener* tf_; // Pointer to our own listener, managed by new/delete

    // ROS Nodehandle and Publisher/Subscriber
    ros::NodeHandle private_nh_;
    ros::Subscriber polygon_sub_;
    ros::Publisher path_pub_; // Publisher for visualizing the generated path in RViz

    // Stored polygon data
    geometry_msgs::Polygon coverage_polygon_; // As specified by assignment
    bool polygon_received_; // Flag to indicate if a valid polygon has been received

    // Planner parameters
    double robot_radius_; // Parameter for robot radius, determines sweep line spacing

    // Flag to check if the planner has been initialized
    bool initialized_;

    // Callback for the polygon topic
    void polygonCallback(const geometry_msgs::Polygon::ConstPtr& msg);

    // --- Helper functions for coverage path generation ---

    /**
     * @brief Checks if a horizontal line at y=line_y intersects the line segment p1-p2.
     * @param line_y The y-coordinate of the horizontal line.
     * @param p1 The first endpoint of the segment.
     * @param p2 The second endpoint of the segment.
     * @param intersect_x Output parameter: the x-coordinate of the intersection if found.
     * @return True if an intersection point is found within the segment's bounds (inclusive of endpoints), false otherwise.
     */
    bool getHorizontalLineSegmentIntersection(double line_y,
                                              const geometry_msgs::Point32& p1,
                                              const geometry_msgs::Point32& p2,
                                              double& intersect_x) const;

    /**
     * @brief Creates a PoseStamped message with the specified coordinates, yaw, and frame ID.
     * @param x The x-coordinate.
     * @param y The y-coordinate.
     * @param yaw The yaw angle in radians.
     * @param frame_id The frame ID for the pose header.
     * @return The created PoseStamped message.
     */
    geometry_msgs::PoseStamped createPoseStamped(double x, double y, double yaw, const std::string& frame_id) const;

    // Helper function to check if a pose is within the polygon (Optional - not used in this basic impl)
    // bool isPoseInPolygon(const geometry_msgs::Pose& pose);
};
} // namespace blm6191_coverage_planner

#endif // COVERAGE_PLANNER_H_