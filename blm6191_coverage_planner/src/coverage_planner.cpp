#include <blm6191_coverage_planner/coverage_planner.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/exceptions.h>

PLUGINLIB_EXPORT_CLASS(blm6191_coverage_planner::CoveragePlanner, nav_core::BaseGlobalPlanner)

namespace blm6191_coverage_planner {

CoveragePlanner::CoveragePlanner() : costmap_ros_(NULL), initialized_(false), polygon_received_(false), tf_(NULL) {
}

CoveragePlanner::~CoveragePlanner() {
    if (tf_) {
        delete tf_;
        tf_ = NULL;
    }
}

void CoveragePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ros_ = costmap_ros;
        private_nh_ = ros::NodeHandle("~/" + name);

        if (!tf_) {
           try {
               tf_ = new tf::TransformListener(ros::Duration(30.0));
           } catch (tf::TransformException& ex) {
               ROS_FATAL("TransformListener oluşturulurken hata: %s", ex.what());
           }
        } else {
             ROS_WARN("TF Listener zaten oluşturulmuş?");
        }

        private_nh_.param("robot_radius", robot_radius_, 0.25);
        std::string polygon_topic = "coverage_polygon";
        private_nh_.param("polygon_topic", polygon_topic, polygon_topic);

        ros::NodeHandle nh;
        polygon_sub_ = nh.subscribe<geometry_msgs::Polygon>(
            polygon_topic, 1, &CoveragePlanner::polygonCallback, this);

        path_pub_ = private_nh_.advertise<nav_msgs::Path>("global_coverage_path", 1);

        ROS_INFO("Kapsama Alanı Global Planlayıcısı başlatıldı.");
        initialized_ = true;
    } else {
        ROS_WARN("Kapsama Alanı Global Planlayıcısı zaten başlatılmış.");
    }
}

void CoveragePlanner::polygonCallback(const geometry_msgs::Polygon::ConstPtr& msg) {
    coverage_polygon_ = *msg;
    ROS_INFO("Kapsama alanı poligonu alındı, %zu nokta içeriyor.", coverage_polygon_.points.size());

    if (coverage_polygon_.points.size() != 4) {
         ROS_WARN("Alınan poligon tam olarak 4 nokta içermeli (dörtgen), fakat %zu nokta alındı. Bu poligon kapsama planlaması için kullanılamaz.", coverage_polygon_.points.size());
         polygon_received_ = false;
    } else {
         ROS_INFO("Geçerli 4 noktalı poligon alındı. Planlayıcı makePlan çağrıldığında plan oluşturmaya hazır.");
         polygon_received_ = true;
    }
}

bool CoveragePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("Kapsama Alanı Global Planlayıcısı henüz başlatılmadı");
        return false;
    }

    if (!polygon_received_) {
        ROS_WARN("Henüz geçerli 4 noktalı kapsama poligonu alınmadı. Plan oluşturulamıyor.");
        return false;
    }

    if (!tf_) {
        ROS_ERROR("TF dinleyicisi başlatılmadı. Plan oluşturulamıyor.");
        return false;
    }

    std::string costmap_frame = costmap_ros_->getGlobalFrameID();

    geometry_msgs::PoseStamped start_in_costmap_frame;
    if (start.header.frame_id == costmap_frame) {
        start_in_costmap_frame = start;
    } else {
        if (!tf_->waitForTransform(costmap_frame, start.header.frame_id, start.header.stamp, ros::Duration(1.0))) {
             ROS_ERROR("Başlangıç pozisyonu %s'den %s'ye dönüştürülemiyor. Dönüşüm mevcut değil.",
                       start.header.frame_id.c_str(), costmap_frame.c_str());
             return false;
        }
        try {
            tf_->transformPose(costmap_frame, start, start_in_costmap_frame);
             ROS_INFO("Başlangıç pozisyonu %s'den %s'ye dönüştürüldü.", start.header.frame_id.c_str(), costmap_frame.c_str());
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Başlangıç pozisyonu %s'den %s'ye dönüştürülemedi: %s",
                      start.header.frame_id.c_str(), costmap_frame.c_str(), ex.what());
            return false;
        }
    }

    ROS_INFO("Poligon noktalarının costmap koordinat sisteminde (%s) olduğu varsayılıyor çünkü geometry_msgs::Polygon'un başlığı yok.", costmap_frame.c_str());

    plan.clear();

    ROS_INFO("Kapsama Alanı Global Planı hesaplanıyor...");
    ROS_INFO("Robot yarıçapı: %f m (tarama genişliği: %f m)", robot_radius_, 2.0 * robot_radius_);

    const double coverage_width = 2.0 * robot_radius_;
    const double EPSILON = 1e-6;

    std::vector<geometry_msgs::Point32> polygon_points;
    for(const auto& p : coverage_polygon_.points) {
        polygon_points.push_back(p);
    }

    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    for (const auto& p : polygon_points) {
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
    }

    bool sweep_forward = true;

    for (double current_y = min_y + robot_radius_;
         current_y <= max_y - robot_radius_ + EPSILON;
         current_y += coverage_width)
    {
        std::vector<double> intersections_x;
        double intersect_x;

        if (getHorizontalLineSegmentIntersection(current_y, polygon_points[0], polygon_points[1], intersect_x)) {
             intersections_x.push_back(intersect_x);
        }
        if (getHorizontalLineSegmentIntersection(current_y, polygon_points[1], polygon_points[2], intersect_x)) {
             intersections_x.push_back(intersect_x);
        }
        if (getHorizontalLineSegmentIntersection(current_y, polygon_points[2], polygon_points[3], intersect_x)) {
             intersections_x.push_back(intersect_x);
        }
        if (getHorizontalLineSegmentIntersection(current_y, polygon_points[3], polygon_points[0], intersect_x)) {
             intersections_x.push_back(intersect_x);
        }

        if (intersections_x.size() >= 2 && intersections_x.size() % 2 == 0) {
            std::sort(intersections_x.begin(), intersections_x.end());

            for(size_t i = 0; i < intersections_x.size(); i += 2) {
                double x1 = intersections_x[i];
                double x2 = intersections_x[i+1];

                if (std::abs(x2 - x1) < EPSILON) {
                    continue;
                }

                double yaw = sweep_forward ? 0.0 : M_PI;

                if (sweep_forward) {
                    plan.push_back(createPoseStamped(x1, current_y, yaw, costmap_frame));
                    plan.push_back(createPoseStamped(x2, current_y, yaw, costmap_frame));
                } else {
                    plan.push_back(createPoseStamped(x2, current_y, yaw, costmap_frame));
                    plan.push_back(createPoseStamped(x1, current_y, yaw, costmap_frame));
                }
            }

            sweep_forward = !sweep_forward;

        } else {
             if (!intersections_x.empty()) {
                 ROS_WARN_THROTTLE(1.0, "y=%f'deki yatay çizgi %zu kenarla kesişti. Güvenilir eşleştirme için çift sayıda >= 2 bekleniyordu. Bu tarama atlanıyor.", current_y, intersections_x.size());
             }
        }
    }

    if (!plan.empty()) {
         nav_msgs::Path viz_path;
         viz_path.header.frame_id = costmap_frame;
         viz_path.header.stamp = ros::Time::now();
         viz_path.poses = plan;
         path_pub_.publish(viz_path);
         ROS_INFO("Kapsama yolu %zu nokta ile yayınlandı.", plan.size());
    } else {
         ROS_WARN("Oluşturulan plan boş. Poligon boyutunu ve robot yarıçapını poligon büyüklüğüne göre kontrol edin.");
    }

    if (!plan.empty()) {
        ROS_INFO("Kapsama Alanı Global Planı %zu nokta ile hesaplandı.", plan.size());
        return true;
    } else {
        ROS_WARN("Kapsama planı oluşturulamadı. Sonuç planı boş. Poligon girişini kontrol edin (4 noktalı dörtgen olmalı) veya robot_radius'u kontrol edin. Poligon robot yarıçapı için çok küçük veya ince olabilir, ya da tarama çizgileri kenarlarla düzgün kesişmiyor olabilir.");
        return false;
    }
}

bool CoveragePlanner::getHorizontalLineSegmentIntersection(double line_y,
                                          const geometry_msgs::Point32& p1,
                                          const geometry_msgs::Point32& p2,
                                          double& intersect_x) const {
    const double EPSILON = 1e-6;

     double min_seg_y = std::min(p1.y, p2.y);
     double max_seg_y = std::max(p1.y, p2.y);

    bool y_between = (line_y >= min_seg_y - EPSILON && line_y <= max_seg_y + EPSILON);

    if (!y_between) {
        return false;
    }

    if (std::abs(p1.x - p2.x) < EPSILON) {
        intersect_x = p1.x;
        return true;
    }

    if (std::abs(p1.y - p2.y) < EPSILON) {
        return false;
    }

    intersect_x = p1.x + (line_y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);

    bool x_between = (intersect_x >= std::min(p1.x, p2.x) - EPSILON &&
                      intersect_x <= std::max(p1.x, p2.x) + EPSILON);

    return x_between;
}

geometry_msgs::PoseStamped CoveragePlanner::createPoseStamped(double x, double y, double yaw, const std::string& frame_id) const {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return pose;
}

} // namespace blm6191_coverage_planner