#pragma once
#include <map_msgs/OccupancyGridUpdate.h>
#include <mrrtpe_msgs/Obstacle.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>

#include <costmap_converter/costmap_converter_interface.h>
#include <pluginlib/class_loader.h>
#include <obstacle_extractor/marker_publisher.hpp>

namespace mrrtpe::ugv_obstacle_extractor {

class ObstacleExtractor {
  public:
    ObstacleExtractor(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  private:
    void extract(const nav_msgs::OccupancyGrid& msg);
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
    void mapUpdatesCallback(const map_msgs::OccupancyGridUpdateConstPtr& msg);


    pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> converter_loader_;
    boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> converter_;
    costmap_2d::Costmap2D map_;
    int occupied_min_value_;
    bool publish_marker_;
    MarkerPublisher marker_publisher_;

    nav_msgs::OccupancyGridConstPtr map_ptr_;

    ros::Subscriber map_sub_;
    ros::Subscriber map_update_sub_;
    ros::Publisher obs_list_pub_;
    ros::Publisher map_pub_;
};

}  // namespace mrrtpe::ugv_obstacle_extractor
