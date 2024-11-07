#pragma once

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
namespace mrrtpe_simulation {
class IntractMarker {
  public:
    IntractMarker(const std::string& name, const std::string& frame_id, interactive_markers::InteractiveMarkerServer::FeedbackCallback cb) 
    : server_(name){

        // create an interactive marker for our server
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = frame_id;
        int_marker.header.stamp = ros::Time::now();
        int_marker.name = name + "_marker";
        int_marker.description = "plane control";

        // create a grey box marker
        visualization_msgs::Marker box_marker;
        box_marker.type = visualization_msgs::Marker::CUBE;
        box_marker.scale.x = 0.45;
        box_marker.scale.y = 0.45;
        box_marker.scale.z = 0.45;
        box_marker.color.r = 0.5;
        box_marker.color.g = 0.5;
        box_marker.color.b = 0.5;
        box_marker.color.a = 1.0;

        // create a non-interactive control which contains the box
        visualization_msgs::InteractiveMarkerControl box_control;
        box_control.always_visible = true;
        box_control.markers.push_back(box_marker);

        // add the control to the interactive marker
        int_marker.controls.push_back(box_control);

        // create a control which will move the box
        // this control does not contain any markers,
        // which will cause RViz to insert four arrows
        visualization_msgs::InteractiveMarkerControl plane_control;
        plane_control.name = "move_x";
        plane_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

        // add the control to the interactive marker
        int_marker.controls.push_back(plane_control);
        plane_control.orientation.w = 1;
        plane_control.orientation.x = 0;
        plane_control.orientation.y = 0;
        plane_control.orientation.z = 1;
        plane_control.name = "move_y";
        plane_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

        // add the control to the interactive marker
        int_marker.controls.push_back(plane_control);

        // add the interactive marker to our collection &
        // tell the server to call cb() when feedback arrives for it
        server_.insert(int_marker, cb);

        // 'commit' changes and send to all clients
        server_.applyChanges();
    }
    private:
      interactive_markers::InteractiveMarkerServer server_;
};

}  // namespace mrrtpe_simulation
