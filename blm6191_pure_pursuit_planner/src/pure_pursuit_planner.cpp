#include <blm6191_pure_pursuit_planner/pure_pursuit_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

PLUGINLIB_EXPORT_CLASS(blm6191_pure_pursuit_planner::PurePursuitPlanner, nav_core::BaseLocalPlanner)

namespace blm6191_pure_pursuit_planner {

PurePursuitPlanner::PurePursuitPlanner() : tf_(NULL), costmap_ros_(NULL), initialized_(false), goal_reached_(false), current_waypoint_index_(0) {
}

PurePursuitPlanner::~PurePursuitPlanner() {
}

void PurePursuitPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        tf_ = tf;
        costmap_ros_ = costmap_ros;

        private_nh_ = ros::NodeHandle("~/" + name);

        private_nh_.param("lookahead_distance", lookahead_distance_, 0.4);
        private_nh_.param("max_linear_velocity", max_linear_velocity_, 0.5);
        private_nh_.param("max_angular_velocity", max_angular_velocity_, 1.0);
        private_nh_.param("goal_tolerance", goal_tolerance_, 0.1);
        private_nh_.param("transform_tolerance", transform_tolerance_, 0.1);

        ROS_INFO("Pure Pursuit Local Planner initialized with lookahead: %f, max_vel_x: %f, max_vel_angular: %f",
                 lookahead_distance_, max_linear_velocity_, max_angular_velocity_);

        initialized_ = true;
    } else {
        ROS_WARN("Pure Pursuit Local Planner has already been initialized.");
    }
}

bool PurePursuitPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (!initialized_) {
        ROS_ERROR("Pure Pursuit Local Planner has not been initialized.");
        return false;
    }
    global_plan_ = orig_global_plan;
    current_waypoint_index_ = 0;
    goal_reached_ = false;
    ROS_INFO("Received global plan with %zu points. Starting Pure Pursuit tracking from index 0.", global_plan_.size());
    return true;
}

bool PurePursuitPlanner::isGoalReached() {
    if (!initialized_) {
        ROS_ERROR("Pure Pursuit Local Planner has not been initialized.");
        return false;
    }
    if (global_plan_.empty()) {
        return true;
    }

    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose)) {
        ROS_ERROR("Failed to get robot pose for goal check.");
        return false;
    }

    geometry_msgs::PoseStamped goal_pose = global_plan_.back();

    double dist_to_goal = distance(robot_pose.pose.position, goal_pose.pose.position);

    if (dist_to_goal < goal_tolerance_) {
        goal_reached_ = true;
        ROS_INFO_THROTTLE(1.0, "Goal reached! Distance to final plan point: %f (tolerance: %f)", dist_to_goal, goal_tolerance_);
        return true;
    }

    return false;
}

bool PurePursuitPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!initialized_) {
        ROS_ERROR("Pure Pursuit Local Planner has not been initialized.");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return false;
    }
    if (isGoalReached()) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        ROS_INFO_THROTTLE(1.0, "Goal reached, sending zero velocity.");
        return true;
    }
    if (global_plan_.empty()) {
         ROS_ERROR("Received empty global plan.");
         cmd_vel.linear.x = 0.0;
         cmd_vel.angular.z = 0.0;
         return false;
    }

    geometry_msgs::PoseStamped lookahead_pose_global;

    if (!findLookaheadPoint(lookahead_pose_global)) {
        ROS_WARN_THROTTLE(1.0, "Could not find a lookahead point on the global plan, but goal not reached. Stopping.");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return false;
    }

    geometry_msgs::PoseStamped lookahead_pose_robot_frame;
    std::string base_frame_id = costmap_ros_->getBaseFrameID();

    try {
        geometry_msgs::TransformStamped transform = tf_->lookupTransform(
            base_frame_id, lookahead_pose_global.header.frame_id,
            lookahead_pose_global.header.stamp, ros::Duration(transform_tolerance_));
        tf2::doTransform(lookahead_pose_global, lookahead_pose_robot_frame, transform);
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Failed to transform lookahead point from %s to %s frame: %s. Stopping.",
                  lookahead_pose_global.header.frame_id.c_str(), base_frame_id.c_str(), ex.what());
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return false;
    }

    double curvature = calculateCurvature(lookahead_pose_robot_frame);

    cmd_vel.linear.x = max_linear_velocity_;
    cmd_vel.angular.z = cmd_vel.linear.x * curvature;

    if (std::abs(cmd_vel.angular.z) > max_angular_velocity_) {
         cmd_vel.angular.z = std::copysign(max_angular_velocity_, cmd_vel.angular.z);
    }

    return true;
}

double PurePursuitPlanner::distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

bool PurePursuitPlanner::findLookaheadPoint(geometry_msgs::PoseStamped& lookahead_point) {
    geometry_msgs::PoseStamped robot_pose_global;
    if (!costmap_ros_->getRobotPose(robot_pose_global)) {
        ROS_ERROR("Failed to get robot pose in findLookaheadPoint.");
        return false;
    }
    geometry_msgs::Point robot_pos_point = robot_pose_global.pose.position;

    double min_dist_from_start = -1.0;
    double max_dist_on_path = 0.0;
    bool found_point = false;

    unsigned int search_start_index = current_waypoint_index_ > 0 ? current_waypoint_index_ - 1 : 0;

    for (unsigned int i = search_start_index; i < global_plan_.size() - 1; ++i) {
        geometry_msgs::Point p1 = global_plan_[i].pose.position;
        geometry_msgs::Point p2 = global_plan_[i+1].pose.position;

        double dx_seg = p2.x - p1.x;
        double dy_seg = p2.y - p1.y;
        double dx_r = p1.x - robot_pos_point.x;
        double dy_r = p1.y - robot_pos_point.y;

        double a = dx_seg*dx_seg + dy_seg*dy_seg;
        double b = 2 * (dx_r*dx_seg + dy_r*dy_seg);
        double c = dx_r*dx_r + dy_r*dy_r - lookahead_distance_*lookahead_distance_;

        double discriminant = b*b - 4*a*c;

        if (discriminant < 0) {
            continue;
        }

        double u1 = (-b + std::sqrt(discriminant)) / (2*a);
        double u2 = (-b - std::sqrt(discriminant)) / (2*a);

        std::vector<double> valid_us;

        if (u1 >= -1e-6 && u1 <= 1.0 + 1e-6) {
             geometry_msgs::Point intersection_point;
             intersection_point.x = p1.x + u1 * dx_seg;
             intersection_point.y = p1.y + u1 * dy_seg;

             geometry_msgs::PoseStamped intersection_pose;
             intersection_pose.header.frame_id = global_plan_[i].header.frame_id;
             intersection_pose.pose.position = intersection_point;
             intersection_pose.pose.orientation.w = 1.0;

             geometry_msgs::PoseStamped intersection_robot_frame;

             try {
                 geometry_msgs::TransformStamped transform = tf_->lookupTransform(
                     costmap_ros_->getBaseFrameID(), intersection_pose.header.frame_id,
                     robot_pose_global.header.stamp, ros::Duration(transform_tolerance_));
                 tf2::doTransform(intersection_pose, intersection_robot_frame, transform);

                 if (intersection_robot_frame.pose.position.x > -0.05) {
                     valid_us.push_back(u1);
                 }
             } catch (tf2::TransformException& ex) {
                 ROS_ERROR_THROTTLE(1.0, "TF Error checking intersection point u1: %s", ex.what());
             }
        }

         if (u2 >= -1e-6 && u2 <= 1.0 + 1e-6) {
             geometry_msgs::Point intersection_point;
            intersection_point.x = p1.x + u2 * dx_seg;
            intersection_point.y = p1.y + u2 * dy_seg;

             geometry_msgs::PoseStamped intersection_pose;
            intersection_pose.header.frame_id = global_plan_[i].header.frame_id;
            intersection_pose.pose.position = intersection_point;
             intersection_pose.pose.orientation.w = 1.0;

            geometry_msgs::PoseStamped intersection_robot_frame;

             try {
                 geometry_msgs::TransformStamped transform = tf_->lookupTransform(
                     costmap_ros_->getBaseFrameID(), intersection_pose.header.frame_id,
                     robot_pose_global.header.stamp, ros::Duration(transform_tolerance_));
                 tf2::doTransform(intersection_pose, intersection_robot_frame, transform);

                 if (intersection_robot_frame.pose.position.x > -0.05) {
                     valid_us.push_back(u2);
                 }
             } catch (tf2::TransformException& ex) {
                 ROS_ERROR_THROTTLE(1.0, "TF Error checking intersection point u2: %s", ex.what());
             }
         }

        if (!valid_us.empty()) {
            double best_u = valid_us[0];
            if (valid_us.size() > 1) {
                best_u = std::max(valid_us[0], valid_us[1]);
            }

            geometry_msgs::Point final_intersection_point;
            final_intersection_point.x = p1.x + best_u * dx_seg;
            final_intersection_point.y = p1.y + best_u * dy_seg;
            final_intersection_point.z = 0.0;

            current_waypoint_index_ = i;

            lookahead_point.header = global_plan_[i].header;
            lookahead_point.pose.position = final_intersection_point;
            lookahead_point.pose.orientation.w = 1.0;

            found_point = true;
            break;
        }
    }

    if (!found_point && global_plan_.size() > 0) {
        geometry_msgs::PoseStamped& last_pose = global_plan_.back();
        double dist_to_last = distance(robot_pos_point, last_pose.pose.position);

        if (dist_to_last <= lookahead_distance_) {
             lookahead_point = last_pose;
             current_waypoint_index_ = global_plan_.size() - 1;
             found_point = true;
        }
    }

    return found_point;
}

double PurePursuitPlanner::calculateCurvature(const geometry_msgs::PoseStamped& lookahead_point_robot_frame) const {
    if (lookahead_distance_ < 1e-6) {
        ROS_WARN_THROTTLE(1.0, "Lookahead distance is too small (%f), returning 0 curvature.", lookahead_distance_);
        return 0.0;
    }

    double y = lookahead_point_robot_frame.pose.position.y;
    double ld_sq = lookahead_distance_ * lookahead_distance_;

    double curvature = 2.0 * y / ld_sq;

    return curvature;
}

} // namespace blm6191_pure_pursuit_planner