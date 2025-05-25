#include <blm6191_coverage_planner/coverage_planner.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
// std_msgs/String.h is not strictly needed, removed for clarity
#include <cmath> // For M_PI, std::abs, std::min, std::max etc.
#include <algorithm> // For std::sort
#include <limits>    // For std::numeric_limits
#include <vector>    // For std::vector
#include <geometry_msgs/Point32.h> // Need this explicitly for Point32
#include <nav_msgs/Path.h> // Need this explicitly for Path
// tf/transform_listener.h is already included in .h
#include <tf/transform_datatypes.h> // Added for tf::createQuaternionMsgFromYaw (part of tf)
#include <tf/exceptions.h> // Added for tf::TransformException


// Register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(blm6191_coverage_planner::CoveragePlanner, nav_core::BaseGlobalPlanner)

namespace blm6191_coverage_planner {

CoveragePlanner::CoveragePlanner() : costmap_ros_(NULL), initialized_(false), polygon_received_(false), tf_(NULL) {
    // tf_ will be initialized in the initialize method
}

CoveragePlanner::~CoveragePlanner() {
    // Delete our own TF listener instance created with 'new'
    if (tf_) {
        delete tf_;
        tf_ = NULL;
    }
    // costmap_ros_ is NOT owned by this class, do not delete
}

void CoveragePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ros_ = costmap_ros;
        // costmap_ = costmap_ros_->getCostmap(); // costmap_ pointer not used in this basic version

        private_nh_ = ros::NodeHandle("~/" + name); // Get private namespace for parameters

        // Initialize our own TF listener. Using 'new' allows us to initialize it here
        // within the plugin lifecycle. A buffer size (e.g., 30 seconds) is recommended
        // to handle transforms that are slightly in the past or future.
        if (!tf_) { // Ensure it's only created once
           try {
               tf_ = new tf::TransformListener(ros::Duration(30.0)); // 30 seconds cache size
           } catch (tf::TransformException& ex) {
               ROS_FATAL("Error creating TransformListener: %s", ex.what());
               // If TF listener creation fails, the planner cannot function.
               // Note: In some setups, TF might not be fully ready when initialize is called.
               // You might add a ROS_WARN here and handle tf_ == NULL later, but for a global planner needing transforms,
               // failing initialization is often safer if TF is critical.
           }
        } else {
             ROS_WARN("TF Listener already created?");
        }

        // Load parameters
        // Default robot_radius is 0.25m, resulting in 0.5m swath width
        private_nh_.param("robot_radius", robot_radius_, 0.25);
        std::string polygon_topic = "coverage_polygon"; // Default polygon topic name
        private_nh_.param("polygon_topic", polygon_topic, polygon_topic);

        // Subscribe to the polygon topic
        // Use a global node handle (nh) if the topic is global (/coverage_polygon)
        // Use private_nh_ if the topic is relative (~/coverage_polygon or planner_name/coverage_polygon)
        // Ödevde poligon topic'inin adı belirtilmemiş, genellikle global olur.
        ros::NodeHandle nh; // Global node handle
        polygon_sub_ = nh.subscribe<geometry_msgs::Polygon>(
            polygon_topic, 1, &CoveragePlanner::polygonCallback, this);


        // Publisher for visualizing the generated path in RViz
        path_pub_ = private_nh_.advertise<nav_msgs::Path>("global_coverage_path", 1);

        ROS_INFO("Coverage Global Planner initialized.");
        initialized_ = true;
    } else {
        ROS_WARN("Coverage Global Planner has already been initialized.");
    }
}

void CoveragePlanner::polygonCallback(const geometry_msgs::Polygon::ConstPtr& msg) {
    coverage_polygon_ = *msg; // Copy the polygon data
    ROS_INFO("Coverage polygon received with %zu points.", coverage_polygon_.points.size());

    // --- Strict check for exactly 4 points (quadrilateral) as requested ---
    if (coverage_polygon_.points.size() != 4) {
         ROS_WARN("Received polygon must have exactly 4 points (a quadrilateral), but got %zu. Cannot use this polygon for coverage planning.", coverage_polygon_.points.size());
         polygon_received_ = false; // Invalidate if not 4 points
    } else {
         // Optional: Add check for convexity here if needed.
         // The current intersection logic implicitly assumes convexity for reliable pairing.
         ROS_INFO("Valid 4-point polygon received. Planner is ready to generate plan when makePlan is called.");
         polygon_received_ = true;
    }
}

bool CoveragePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal, // Goal is used as a trigger
                               std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("Coverage Global Planner has not been initialized");
        return false;
    }

    if (!polygon_received_) {
        ROS_WARN("No valid 4-point coverage polygon received yet. Cannot make plan.");
        return false;
    }

    // Check if TF listener was successfully created
    if (!tf_) {
        ROS_ERROR("TF listener is not initialized. Cannot make plan.");
        return false;
    }


    // Get the costmap frame ID. The plan should be generated in this frame.
    std::string costmap_frame = costmap_ros_->getGlobalFrameID();

    // Ensure the start pose is in the costmap frame (e.g., map frame) for potential use
    // (Although this planner doesn't strictly use the start pose coordinates for path *generation*,
    // transforming it is good practice if you ever need it or add start-point alignment).
    geometry_msgs::PoseStamped start_in_costmap_frame;
    if (start.header.frame_id == costmap_frame) {
        start_in_costmap_frame = start;
    } else {
        // Wait for the transform to become available
        if (!tf_->waitForTransform(costmap_frame, start.header.frame_id, start.header.stamp, ros::Duration(1.0))) { // Wait up to 1 second
             ROS_ERROR("Cannot transform start pose from %s to %s. Transform not available.",
                       start.header.frame_id.c_str(), costmap_frame.c_str());
             return false; // Cannot proceed if transform is needed but fails
        }
        try {
            tf_->transformPose(costmap_frame, start, start_in_costmap_frame);
             ROS_INFO("Transformed start pose from %s to %s for potential use.", start.header.frame_id.c_str(), costmap_frame.c_str());
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Failed to transform start pose from %s to %s: %s",
                      start.header.frame_id.c_str(), costmap_frame.c_str(), ex.what());
            return false;
        }
    }


    // Since geometry_msgs::Polygon doesn't have a header, we must assume its points are already in the costmap frame.
    // If it were PolygonStamped, we would check and transform its header.
    // **CRITICAL:** If the polygon is published in a frame different from the costmap frame,
    // this assumption will lead to incorrect path generation relative to the map/robot.
    ROS_INFO("Assuming polygon points are already in the costmap frame (%s) as geometry_msgs::Polygon has no header.", costmap_frame.c_str());

    plan.clear(); // Clear any previous plan

    ROS_INFO("Calculating Coverage Global Plan...");
    ROS_INFO("Using robot radius: %f m (swath width: %f m)", robot_radius_, 2.0 * robot_radius_);

    // --- Horizontal Sweep Coverage Algorithm ---
    const double coverage_width = 2.0 * robot_radius_;
    const double EPSILON = 1e-6; // Small value for floating point comparisons

    // Get the 4 polygon points
    // Make a copy to avoid modifying the stored polygon_
    std::vector<geometry_msgs::Point32> polygon_points;
    for(const auto& p : coverage_polygon_.points) {
        polygon_points.push_back(p);
    }

    // Find min and max Y coordinates of the polygon points
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    for (const auto& p : polygon_points) {
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
    }

    // Determine sweep direction (alternating)
    // You could make this start direction configurable (e.g., start left-to-right always)
    bool sweep_forward = true; // true: left-to-right (yaw 0), false: right-to-left (yaw PI)

    // Iterate through y-coordinates with step = coverage_width
    // The sweep lines represent the Y coordinate of the center of the robot's path.
    // To cover the area fully, the first sweep line should be at min_y + radius,
    // and the last at max_y - radius.
    // We iterate from min_y + robot_radius_ up to max_y - robot_radius_ + epsilon.
    for (double current_y = min_y + robot_radius_;
         current_y <= max_y - robot_radius_ + EPSILON; // Add epsilon to ensure the last swath is included
         current_y += coverage_width)
    {

        std::vector<double> intersections_x; // Store x-coordinates of intersections for the current y

        // Check intersection of the horizontal line (y=current_y) with each of the 4 polygon edges.
        // The polygon edges are defined by the sequence of points: (p0, p1), (p1, p2), (p2, p3), (p3, p0).
        double intersect_x;

        // Edge 0-1
        if (getHorizontalLineSegmentIntersection(current_y, polygon_points[0], polygon_points[1], intersect_x)) {
             intersections_x.push_back(intersect_x);
        }
        // Edge 1-2
        if (getHorizontalLineSegmentIntersection(current_y, polygon_points[1], polygon_points[2], intersect_x)) {
             intersections_x.push_back(intersect_x);
        }
        // Edge 2-3
        if (getHorizontalLineSegmentIntersection(current_y, polygon_points[2], polygon_points[3], intersect_x)) {
             intersections_x.push_back(intersect_x);
        }
        // Edge 3-0
        if (getHorizontalLineSegmentIntersection(current_y, polygon_points[3], polygon_points[0], intersect_x)) {
             intersections_x.push_back(intersect_x);
        }

        // For a simple convex quadrilateral, a horizontal line should ideally intersect exactly 2 edges
        // that define the horizontal extent at that y.
        // If the line passes exactly through a vertex, we might get more (an even number).
        // We sort the intersection points by their x-coordinate and process them in pairs.
        if (intersections_x.size() >= 2 && intersections_x.size() % 2 == 0) {
            std::sort(intersections_x.begin(), intersections_x.end()); // Sort x-coordinates (left to right)

            // Process pairs of sorted intersections to get sweep segments.
            for(size_t i = 0; i < intersections_x.size(); i += 2) {
                double x1 = intersections_x[i];
                double x2 = intersections_x[i+1];

                 // Filter out very short segments that might arise from floating point noise or near-vertex hits
                if (std::abs(x2 - x1) < EPSILON) {
                    // ROS_DEBUG("Skipping very short segment at y=%f from (%f, %f).", current_y, x1, x2);
                    continue;
                }

                // Determine yaw based on sweep direction (0 radians for right, M_PI radians for left).
                // Assumes ROS standard conventions (+X is forward, +Y is left, yaw=0 is facing +X).
                double yaw = sweep_forward ? 0.0 : M_PI;

                // Add points to the plan based on the sweep direction (start point then end point of the segment).
                // Ensure poses are created with the correct frame ID (costmap_frame).
                if (sweep_forward) {
                    plan.push_back(createPoseStamped(x1, current_y, yaw, costmap_frame)); // Start of sweep segment (left)
                    plan.push_back(createPoseStamped(x2, current_y, yaw, costmap_frame)); // End of sweep segment (right)
                } else {
                    plan.push_back(createPoseStamped(x2, current_y, yaw, costmap_frame)); // Start of sweep segment (right)
                    plan.push_back(createPoseStamped(x1, current_y, yaw, costmap_frame)); // End of sweep segment (left)
                }
            }

            // Switch sweep direction for the next swath to create the zigzag pattern.
            sweep_forward = !sweep_forward;

        } else {
             // This line did not intersect an even number of edges greater than or equal to 2.
             // For a simple convex quad, this is unexpected unless the line is exactly tangent to a vertex
             // or horizontal edges cause issues. A more robust planner would handle these cases.
             if (!intersections_x.empty()) {
                 // Log a warning if intersections were found but couldn't be paired correctly.
                 ROS_WARN_THROTTLE(1.0, "Horizontal line at y=%f intersected %zu edges. Expected an even number >= 2 for reliable pairing. Skipping this swath.", current_y, intersections_x.size());
             }
             // If intersections_x is empty, the sweep line is outside the vertical range defined by the polygon,
             // which is expected for lines below min_y or above max_y, or within "gaps" for non-convex shapes.
             // No warning needed for zero intersections.
        }
    }

    // --- Post-processing / Optional Additions ---
    // The generated plan is a sequence of points for the robot to visit in order.
    // For a full navigation task, you might want to:
    // 1. Add the start pose as the very first point (`plan.insert(plan.begin(), start_in_costmap_frame);`)
    // 2. Find the point in the generated plan that is nearest to the start pose and reorder the path segments
    //    so the robot starts near its current location and covers the area efficiently. This adds significant complexity.
    // For Soru 1, just generating the path within the polygon is likely sufficient.

    // Optional: Publish the generated path for visualization in RViz
    if (!plan.empty()) {
         nav_msgs::Path viz_path;
         viz_path.header.frame_id = costmap_frame; // The frame of all points in the plan
         viz_path.header.stamp = ros::Time::now(); // Timestamp the path
         viz_path.poses = plan; // Copy the generated poses
         path_pub_.publish(viz_path); // Publish for RViz
         ROS_INFO("Published coverage path with %zu points.", plan.size());
    } else {
         ROS_WARN("Generated plan is empty. Check polygon size and robot radius relative to polygon extent.");
    }


    // Return true if a plan was generated (even if empty due to filter/errors)
    // or false if fundamental issues prevented planning (e.g., no polygon, no TF).
    // It's better to return true if the planning logic ran but produced an empty path
    // vs. false if the planner couldn't even attempt planning.
    // For this context, let's return true only if the plan has points.
    if (!plan.empty()) {
        ROS_INFO("Coverage Global Plan calculated with %zu points.", plan.size());
        return true; // Plan found and generated
    } else {
        ROS_WARN("Failed to generate coverage plan. Resulting plan is empty. Check polygon input (must be 4-point quadrilateral) or robot_radius. Polygon may be too small or thin for robot radius, or sweep lines might not intersect any edges cleanly.");
        return false; // No plan generated
    }
}

// --- Helper function implementations ---

// Checks if a horizontal line at y=line_y intersects the line segment p1-p2.
// If yes, returns true and sets intersect_x to the x-coordinate of the intersection.
bool CoveragePlanner::getHorizontalLineSegmentIntersection(double line_y,
                                          const geometry_msgs::Point32& p1,
                                          const geometry_msgs::Point32& p2,
                                          double& intersect_x) const {
    const double EPSILON = 1e-6; // Tolerance for floating point comparisons

    // Check if the horizontal line is within the y-range of the segment (with tolerance)
    // Use a slightly larger tolerance for the line_y check compared to standard intersection
    // to catch cases where the line passes very close to a segment endpoint.
     double min_seg_y = std::min(p1.y, p2.y);
     double max_seg_y = std::max(p1.y, p2.y);

    bool y_between = (line_y >= min_seg_y - EPSILON && line_y <= max_seg_y + EPSILON);

    if (!y_between) {
        return false; // Line does not cross the vertical extent of the segment
    }

    // Handle vertical segments (where x is constant). Delta X is close to zero.
    if (std::abs(p1.x - p2.x) < EPSILON) {
        // Segment is vertical or nearly vertical.
        // Since y_between is true, the horizontal line must intersect this vertical segment
        // at the segment's x-coordinate.
        intersect_x = p1.x; // The x-coordinate is constant for this vertical segment
        return true;
    }

    // Handle nearly horizontal segments (where y is constant). Delta Y is close to zero.
    if (std::abs(p1.y - p2.y) < EPSILON) {
        // Segment is horizontal or nearly horizontal.
        // If the horizontal line is the same as the segment's y (within tolerance), it technically intersects
        // along the whole segment. However, for the sweep line algorithm, we are primarily interested in
        // intersections with *non-horizontal* edges that define the polygon's horizontal extent at a given y.
        // Counting intersections with horizontal segments that lie *exactly* on the sweep line
        // can lead to issues (infinite points, incorrect pairing).
        // Returning false here means we ignore horizontal segments lying on the sweep line for intersection purposes.
        return false;
    }

    // Handle sloped segments
    // Use the line equation: (y - p1.y) / (x - p1.x) = (p2.y - p1.y) / (p2.x - p1.x)
    // Solve for x when y = line_y
    // x - p1.x = (line_y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y)
    // x = p1.x + (line_y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y)
    intersect_x = p1.x + (line_y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);

    // Check if the calculated intersection point's x-coordinate falls within the x-range of the segment (with tolerance).
    // This ensures the intersection occurs *on the segment*, not just on the infinite line containing the segment.
    bool x_between = (intersect_x >= std::min(p1.x, p2.x) - EPSILON &&
                      intersect_x <= std::max(p1.x, p2.x) + EPSILON);

    return x_between; // Return true only if both y_between (checked earlier) and x_between are true
}

// Helper function to create a PoseStamped message.
geometry_msgs::PoseStamped CoveragePlanner::createPoseStamped(double x, double y, double yaw, const std::string& frame_id) const {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id; // Set the frame ID (should be costmap frame)
    pose.header.stamp = ros::Time::now(); // Set the timestamp to current ROS time
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0; // Assume 2D planning, z=0.0
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw); // Convert yaw angle (radians) to Quaternion
    return pose;
}


// isPoseInPolygon helper is commented out as not used in this basic impl.
// You would implement this if you needed to check if a point (like start/goal) is inside the polygon.
// bool CoveragePlanner::isPoseInPolygon(const geometry_msgs::Pose& pose) {
//     // Implement point-in-polygon test here (e.g., using Ray Casting or Winding Number algorithm).
//     return false; // Placeholder implementation
// }

} // namespace blm6191_coverage_planner