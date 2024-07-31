#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "costmap_publisher_node");
    ros::NodeHandle nh;

    // Create a Transform Listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Parameters
    std::string global_frame = "map"; // Global frame ID for the costmap
    std::string topic_name = "costmap"; // Topic name to publish the costmap

    // Initialize Costmap2DROS
    costmap_2d::Costmap2DROS costmap_ros("costmap", tfBuffer);

    // Wait until the costmap is fully initialized
    ros::Duration(1.0).sleep(); // Allow some time for the initialization

    // Get the underlying Costmap2D object from Costmap2DROS
    costmap_2d::Costmap2D* costmap = costmap_ros.getCostmap();

    // Costmap2DPublisher setup
    costmap_2d::Costmap2DPublisher costmap_publisher(&nh, costmap, global_frame, topic_name, true);

    // Publishing loop
    ros::Rate rate(1.0); // 1 Hz
    while (ros::ok()) {
        costmap_publisher.publishCostmap();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
