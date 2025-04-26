#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
    explicit CostmapNode();
    
    // Callback functions
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void publishCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Costmap operations
    void initializeCostmap();
    void convertToGrid(float range, float angle, int& x_grid, int& y_grid) const;
    void markObstacle(int x, int y);
    void inflateObstacles();

private:
    robot::CostmapCore costmap_;
    
    // ROS 2 communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // Costmap parameters
    double resolution_;       // meters/cell (e.g., 0.1m)
    int width_;      // in cells
    int height_;     // in cells
    double origin_x_;         // meters (world x coordinate of grid(0,0))
    double origin_y_;         // meters (world y coordinate of grid(0,0))
    float inflation_radius_;  // meters
    int max_cost_;            // maximum cost value (e.g., 100)

    // Removed redundant costmap_2d_ since we're using CostmapCore
};

#endif // COSTMAP_NODE_HPP_