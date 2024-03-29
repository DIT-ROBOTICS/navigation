// plugin
#include <pluginlib/class_list_macros.h>

// include
#include "path_layer/path_layer.h"

PLUGINLIB_EXPORT_CLASS(path_layer_namespace::PathLayer, costmap_2d::Layer)

namespace path_layer_namespace {

PathLayer::PathLayer() {
}

void PathLayer::onInitialize() {
    ros::NodeHandle nh("~/" + name_);

    std::string RobotOdom_CB_TopicName;
    std::string RobotPath_CB_TopicName;
    std::string RivalOdom_CB_TopicName[2];
    std::string Rival_Obstacle_CB_TopicName;
    int temp_OdomType;

    // ---------------- Read YAML parameter ----------------
    nh.param("enabled", enabled_, true);
    nh.param("RobotType", RobotType, 1);
    nh.param("OdomCallbackType", temp_OdomType, 0);

    // Inflation
    nh.param("Inflation/Robot/CostScalingFactor", RobotCostScalingFactor, 10.0);
    nh.param("Inflation/Robot/InscribedRadius", RobotInscribedRadius, 0.1);
    nh.param("Inflation/Robot/InflationRadius", RobotInflationRadius, 0.3);
    nh.param("Inflation/Rival/CostScalingFactor", RivalCostScalingFactor, 10.0);
    nh.param("Inflation/Rival/InscribedRadius", RivalInscribedRadius, 0.1);
    nh.param("Inflation/Rival/InflationRadius", RivalInflationRadius, 0.3);
    nh.param("Inflation/Rival/InscribedRadiusDecline", RivalRadiusDecline, 0.001);

    // Topic
    nh.param<std::string>("Topic/Robot/Odom", RobotOdom_CB_TopicName, "/robot1/odom");
    nh.param<std::string>("Topic/Robot/Path", RobotPath_CB_TopicName, "/move_base/GlobalPlanner/plan");
    nh.param<std::string>("Topic/Rival/Odom1", RivalOdom_CB_TopicName[0], "/RivalOdom_1");
    nh.param<std::string>("Topic/Rival/Odom2", RivalOdom_CB_TopicName[1], "/RivalOdom_2");
    nh.param<std::string>("Topic/Rival/Obstacle", Rival_Obstacle_CB_TopicName, "/RivalObstacle");

    // Timeout
    nh.param("Timeout/Robot/Odom", RobotOdomTimeout, 1.0);
    nh.param("Timeout/Robot/Path", RobotPathTimeout, 1.0);
    nh.param("Timeout/Rival/Odom", RivalOdomTimeout, 1.0);
    nh.param("Timeout/Rival/Obstacle", RivalObstacleTimeout, 1.0);

    // PredictLength
    nh.param("PredictLength/Robot/Path", RobotPredictLength, 1);
    nh.param("PredictLength/Rival/Odom", RivalOdom_PredictTime, 0.1);
    nh.param("PredictLength/Rival/MaxLength", RivalOdom_MaxLength, 1.0);
    nh.param("PredictLength/Rival/Resolution", RivalOdom_Resolution, 0.01);
    // ---------------- Read YAML parameter ----------------

    // Subscriber
    RobotPath_Sub = nh.subscribe(RobotPath_CB_TopicName, 1000, &PathLayer::RobotPath_CB, this);

    switch (temp_OdomType) {
        case 0:
            OdomType = nav_msgs_Odometry;
            RobotOdom_Sub = nh.subscribe(RobotOdom_CB_TopicName, 1000, &PathLayer::RobotOdom_type0_CB, this);
            break;
        case 1:
            OdomType = geometry_msgs_PoseWithCovarianceStamped;
            RobotOdom_Sub = nh.subscribe(RobotOdom_CB_TopicName, 1000, &PathLayer::RobotOdom_type1_CB, this);
            break;
    }

    RivalOdom_Sub[0] = nh.subscribe(RivalOdom_CB_TopicName[0], 1000, &PathLayer::RivalOdom1_CB, this);
    RivalOdom_Sub[1] = nh.subscribe(RivalOdom_CB_TopicName[1], 1000, &PathLayer::RivalOdom2_CB, this);
    RivalObstacle_Sub = nh.subscribe(Rival_Obstacle_CB_TopicName, 1000, &PathLayer::RivalObstacle_CB, this);

    // Init variable
    isRobotPath = isRobotOdom = isRivalOdom[0] = isRivalOdom[1] = isRivalObstacle = false;
    RobotPathLastTime = RivalOdomLastTime[0] = RivalOdomLastTime[1] = RivalObstacleLastTime = ros::Time::now();

    current_ = true;
    default_value_ = costmap_2d::NO_INFORMATION;

    // Resize map
    matchSize();

    // Register layer
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&PathLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void PathLayer::matchSize() {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
              master->getResolution(),
              master->getOriginX(), master->getOriginY());
}

void PathLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level) {
    enabled_ = config.enabled;
}

void PathLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                             double* min_x, double* min_y, double* max_x, double* max_y) {
    if (!enabled_)
        return;

    // Timeout.
    double CurrentTime = ros::Time::now().toSec();
    if (RobotPathTimeout != -1 && CurrentTime - RobotPathLastTime.toSec() > RobotPathTimeout)
        isRobotPath = false;
    if (RobotOdomTimeout != -1 && CurrentTime - RobotOdomLastTime.toSec() > RobotOdomTimeout)
        isRobotOdom = false;
    for (int i = 0; i < 2; i++) {
        if (RivalOdomTimeout != -1 && CurrentTime - RivalOdomLastTime[i].toSec() > RivalOdomTimeout)
            isRivalOdom[i] = false;
    }
    if (RivalObstacleTimeout != -1 && CurrentTime - RivalObstacleLastTime.toSec() > RivalObstacleTimeout)
        isRivalObstacle = false;

    // Get the Costmap lock. (Optional)
    boost::unique_lock<mutex_t> lock(*(getMutex()));

    // Clean up the old costmap.
    resetMaps();

    // Inflation Robot Odom
    if (isRobotOdom) {
        if (OdomType == nav_msgs_Odometry) {
            InflatePoint(RobotOdom_type0.pose.pose.position.x, RobotOdom_type0.pose.pose.position.y, costmap_2d::LETHAL_OBSTACLE, RobotInflationRadius, RobotCostScalingFactor, RobotInscribedRadius);
        } else {
            InflatePoint(RobotOdom_type1.pose.position.x, RobotOdom_type1.pose.position.y, costmap_2d::LETHAL_OBSTACLE, RobotInflationRadius, RobotCostScalingFactor, RobotInscribedRadius);
        }
    }

    // Inflation Robot Path
    if (isRobotPath) {
        InflatePredictPath(ROBOT_TYPE::ROBOT);
    }

    // Add Rival Path to costmap.
    if (isRivalOdom[0]) {
        InflatePredictPath(ROBOT_TYPE::RIVAL1);
    }
    if (isRivalOdom[1]) {
        InflatePredictPath(ROBOT_TYPE::RIVAL2);
    }
    if (isRivalObstacle) {
        InflatePredictPath(ROBOT_TYPE::OBSTACLE);
    }

    *min_x = 0.0;
    *max_x = getSizeInMetersX();
    *min_y = 0.0;
    *max_y = getSizeInMetersY();
}

void PathLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_ || !(isRobotOdom || isRobotPath || isRivalOdom[0] || isRivalOdom[1] || isRivalObstacle))
        return;

    // Get the costmap lock.
    boost::unique_lock<mutex_t> lock(*(getMutex()));

    // updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    updateWithMax(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());
}

void PathLayer::ExpandPointWithCircle(double x, double y, double Radius) {
    for (int angle = 0; angle < 360; angle++) {
        double Rad = angle * M_PI / 180.0;
        double mark_x = x + Radius * cos(Rad);
        double mark_y = y + Radius * sin(Rad);
        unsigned int mx;
        unsigned int my;
        if (worldToMap(mark_x, mark_y, mx, my)) {
            setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
        }
    }
}

void PathLayer::InflatePredictPath(ROBOT_TYPE type) {
    if (type == ROBOT_TYPE::ROBOT) {
        for (int i = 0; i < RobotPredictLength; i++) {
            if (RobotPath.poses.size() <= i)
                break;

            double mark_x = RobotPath.poses[i].pose.position.x;
            double mark_y = RobotPath.poses[i].pose.position.y;
            unsigned int mx;
            unsigned int my;
            if (worldToMap(mark_x, mark_y, mx, my)) {
                InflatePoint(mark_x, mark_y, 252, RobotInflationRadius, RobotCostScalingFactor, RobotInscribedRadius);
            }
        }
    } else if (type == ROBOT_TYPE::RIVAL1 || type == ROBOT_TYPE::RIVAL2) {
        const char idx = (type == ROBOT_TYPE::RIVAL1) ? 0 : 1;

        double mark_x = RivalOdom[idx].pose.pose.position.x;
        double mark_y = RivalOdom[idx].pose.pose.position.y;

        // Rival Object
        InflatePoint(mark_x, mark_y, costmap_2d::LETHAL_OBSTACLE, RivalInflationRadius, RivalCostScalingFactor, RivalInscribedRadius);

        double len = sqrt(pow(RivalOdom[idx].twist.twist.linear.x, 2) + pow(RivalOdom[idx].twist.twist.linear.y, 2)) * RivalOdom_PredictTime;
        if (len == 0.0) {
            return;
        } else if (len > RivalOdom_MaxLength) {
            len = RivalOdom_MaxLength;
        }

        double theta = 0.0;
        if (RivalOdom[idx].twist.twist.linear.x == 0.0) {
            theta = RivalOdom[idx].twist.twist.linear.y >= 0 ? M_PI_2 : -M_PI_2;
        } else {
            theta = std::atan(RivalOdom[idx].twist.twist.linear.y / RivalOdom[idx].twist.twist.linear.x);
            if (RivalOdom[idx].twist.twist.linear.x <= 0) {
                theta += M_PI;
            }
        }

        const double IncX = RivalOdom_Resolution * std::cos(theta);
        const double IncY = RivalOdom_Resolution * std::sin(theta);
        double InflationRadius = RivalInflationRadius;
        double InscribedRadius = RivalInscribedRadius;

        unsigned int mx;
        unsigned int my;
        for (double i = 0.0; i < len; i += RivalOdom_Resolution) {
            mark_x += IncX;
            mark_y += IncY;
            if (worldToMap(mark_x, mark_y, mx, my)) {
                InflatePoint(mark_x, mark_y, 252, InflationRadius, RivalCostScalingFactor, InscribedRadius);
            } else {
                break;
            }
            // InflationRadius *= RivalRadiusDecline;
            InscribedRadius *= RivalRadiusDecline;
        }
    } else if (type == ROBOT_TYPE::OBSTACLE) {
        for (auto Obstacle : RivalObstacle.circles) {
            double mark_x = Obstacle.center.x;
            double mark_y = Obstacle.center.y;

            // Rival Object
            InflatePoint(mark_x, mark_y, costmap_2d::LETHAL_OBSTACLE, RivalInflationRadius, RivalCostScalingFactor, RivalInscribedRadius);

            double len = sqrt(pow(Obstacle.velocity.x, 2) + pow(Obstacle.velocity.y, 2)) * RivalOdom_PredictTime;
            if (len == 0.0) {
                continue;
            } else if (len > RivalOdom_MaxLength) {
                len = RivalOdom_MaxLength;
            }

            double theta = 0.0;
            if (Obstacle.velocity.x == 0.0) {
                theta = Obstacle.velocity.y >= 0 ? M_PI_2 : -M_PI_2;
            } else {
                theta = std::atan(Obstacle.velocity.y / Obstacle.velocity.x);
                if (Obstacle.velocity.x <= 0) {
                    theta += M_PI;
                }
            }

            const double IncX = RivalOdom_Resolution * std::cos(theta);
            const double IncY = RivalOdom_Resolution * std::sin(theta);
            double InflationRadius = RivalInflationRadius;
            double InscribedRadius = RivalInscribedRadius;

            unsigned int mx;
            unsigned int my;
            for (double i = 0.0; i < len; i += RivalOdom_Resolution) {
                mark_x += IncX;
                mark_y += IncY;
                if (worldToMap(mark_x, mark_y, mx, my)) {
                    InflatePoint(mark_x, mark_y, 252, InflationRadius, RivalCostScalingFactor, InscribedRadius);
                } else {
                    break;
                }
                // InflationRadius *= RivalRadiusDecline;
                InscribedRadius *= RivalRadiusDecline;
            }
        }
    }
}

void PathLayer::InflatePoint(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius) {
    // MaxDistance = 6.22258 / CostScalingFactor + InscribedRadius;  // 6.22258 = -ln(0.5/252.0)
    // MaxDistance = (double)(((int)(MaxDistance / resolution_) + resolution_) * resolution_);

    double MaxX = x + InflationRadius;
    double MinX = x - InflationRadius;
    double MaxY;
    double MinY;

    double mark_x = 0.0;
    double mark_y = 0.0;
    unsigned int mx;
    unsigned int my;

    double cost;
    double Distance;

    for (double currentPointX = MinX; currentPointX <= MaxX; currentPointX += resolution_) {
        mark_x = currentPointX;
        MaxY = y + sqrt(pow(InflationRadius, 2) - pow(fabs(currentPointX - x), 2));
        MinY = 2 * y - MaxY;

        for (double currentPointY = MinY; currentPointY <= MaxY; currentPointY += resolution_) {
            mark_y = currentPointY;
            if (worldToMap(mark_x, mark_y, mx, my)) {
                Distance = sqrt(pow(fabs(x - currentPointX), 2) + pow(fabs(y - currentPointY), 2));

                cost = round(252 * exp(-CostScalingFactor * (Distance - InscribedRadius)));
                cost = std::max(std::min(cost, MaxCost), 0.0);

                if (getCost(mx, my) != costmap_2d::NO_INFORMATION) {
                    setCost(mx, my, std::max((unsigned char)cost, getCost(mx, my)));
                } else {
                    setCost(mx, my, cost);
                }
            }
        }
    }
}

// ---------------------------- Callback ---------------------------- //

void PathLayer::RobotPath_CB(const nav_msgs::Path& Path) {
    this->RobotPath = Path;
    isRobotPath = true;
    RobotPathLastTime = ros::Time::now();
}

void PathLayer::RobotOdom_type0_CB(const nav_msgs::Odometry& Odom) {
    this->RobotOdom_type0 = Odom;
    isRobotOdom = true;
    RobotOdomLastTime = ros::Time::now();
}
void PathLayer::RobotOdom_type1_CB(const geometry_msgs::PoseWithCovarianceStamped& Odom) {
    this->RobotOdom_type1 = Odom.pose;
    isRobotOdom = true;
    RobotOdomLastTime = ros::Time::now();
}

void PathLayer::RivalOdom1_CB(const nav_msgs::Odometry& Odom) {
    RivalOdom[0] = Odom;
    isRivalOdom[0] = true;
    RivalOdomLastTime[0] = ros::Time::now();
}

void PathLayer::RivalOdom2_CB(const nav_msgs::Odometry& Odom) {
    RivalOdom[1] = Odom;
    isRivalOdom[1] = true;
    RivalOdomLastTime[1] = ros::Time::now();
}

void PathLayer::RivalObstacle_CB(const obstacle_detector::Obstacles& Obstacle) {
    RivalObstacle = Obstacle;
    isRivalObstacle = true;
    RivalObstacleLastTime = ros::Time::now();
}

}  // namespace path_layer_namespace