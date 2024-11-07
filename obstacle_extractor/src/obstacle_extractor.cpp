#include "obstacle_extractor/obstacle_extractor.hpp"

namespace mrrtpe::ugv_obstacle_extractor {
ObstacleExtractor::ObstacleExtractor(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : map_ptr_(nav_msgs::OccupancyGridConstPtr())
    , marker_publisher_(nh)
    , converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons") {
    std::string converter_plugin = "costmap_converter::CostmapToLinesDBSRANSAC";
    nh_private.param("converter_plugin", converter_plugin, converter_plugin);

    occupied_min_value_ = 100;
    nh_private.param("occupied_min_value", occupied_min_value_, occupied_min_value_);

    publish_marker_ = false;
    nh_private.param("publish_marker", publish_marker_, publish_marker_);

    try {
        converter_ = converter_loader_.createInstance(converter_plugin);
    } catch (const pluginlib::PluginlibException& ex) {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        ros::shutdown();
    }

    if (converter_) {
        converter_->setOdomTopic("odom_topic");
        converter_->initialize(nh_private);
        converter_->setCostmap2D(&map_);
    }

    ROS_INFO_STREAM("Standalone costmap converter:" << converter_plugin << " loaded.");

    map_sub_ =
        nh.subscribe("move_base/global_costmap/costmap", 10, &ObstacleExtractor::mapCallback, this);
    map_update_sub_ = nh.subscribe("move_base/global_costmap/costmap_updates",
        10,
        &ObstacleExtractor::mapUpdatesCallback,
        this);

    obs_list_pub_ = nh.advertise<mrrtpe_msgs::Obstacle>("obs_list", 2);
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("local_obs", 2);
}

void ObstacleExtractor::extract(const nav_msgs::OccupancyGrid& msg) {
    ROS_INFO_ONCE("Got first costmap callback. This message will be printed once");

    if (msg.info.width != map_.getSizeInCellsX() || msg.info.height != map_.getSizeInCellsY() ||
        msg.info.resolution != map_.getResolution()) {
        ROS_INFO("New map format, resizing and resetting map...");
        map_.resizeMap(msg.info.width,
            msg.info.height,
            msg.info.resolution,
            msg.info.origin.position.x,
            msg.info.origin.position.y);
    } else {
        map_.updateOrigin(msg.info.origin.position.x, msg.info.origin.position.y);
    }

    for (std::size_t i = 0; i < msg.data.size(); ++i) {
        unsigned int mx, my;
        map_.indexToCells((unsigned int) i, mx, my);
        map_.setCost(mx, my, msg.data[i] >= occupied_min_value_ ? 255 : 0);
    }

    // convert
    converter_->updateCostmap2D();
    converter_->compute();
    costmap_converter::ObstacleArrayConstPtr obstacles = converter_->getObstacles();

    if (!obstacles)
        return;

    mrrtpe_msgs::Obstacle obs;
    obs.obs_type = mrrtpe_msgs::Obstacle::UNKNOWN;
    obs.pose.header.frame_id = msg.header.frame_id;
    for (auto& obstacle : obstacles->obstacles) {
        for (int j = 0; j < (int) obstacle.polygon.points.size() - 1; ++j) {
            obs.pose.pose.position.x +=
                (obstacle.polygon.points[j].x + obstacle.polygon.points[j + 1].x) / 2.0;
            obs.pose.pose.position.y +=
                (obstacle.polygon.points[j].y + obstacle.polygon.points[j + 1].y) / 2.0;
            obs.pose.pose.position.z += 1;
        }
    }
    if (obs.pose.pose.position.z != 0) {
        obs.pose.pose.position.x /= obs.pose.pose.position.z;
        obs.pose.pose.position.y /= obs.pose.pose.position.z;
        obs.pose.pose.position.z = 0;
        obs_list_pub_.publish(obs);

        if (publish_marker_) {
            marker_publisher_.publishMarkers(msg.header.frame_id, *obstacles, obs.pose);
        }
    }
}

void ObstacleExtractor::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
    map_ptr_ = msg;
}

void ObstacleExtractor::mapUpdatesCallback(const map_msgs::OccupancyGridUpdateConstPtr& msg) {
    const double res = map_ptr_->info.resolution;
    const int width = map_ptr_->info.width;
    const int height = map_ptr_->info.height;

    nav_msgs::OccupancyGrid map;
    map.info.origin = map_ptr_->info.origin;
    map.info.origin.position.x += msg->x * res;
    map.info.origin.position.y += msg->y * res;
    map.info.width = msg->width;
    map.info.height = msg->height;
    map.info.resolution = res;
    map.header = map_ptr_->header;

    int index = (msg->y - 1) * width + msg->x;
    int count = 0;
    for (int i = 0; i < (msg->height); i++) {
        index += map_ptr_->info.width;
        for (int j = index; j < (msg->width) + index; j++) {
            if (int(msg->data[count++] - map_ptr_->data[j]) != 0) {
                auto data = msg->data[count] > 90 ? 100 : 0;
                map.data.push_back(data);
            } else {
                map.data.push_back(0);
            }
        }
    }
    map_pub_.publish(map);
    extract(map);
}

}  // namespace mrrtpe::ugv_obstacle_extractor
