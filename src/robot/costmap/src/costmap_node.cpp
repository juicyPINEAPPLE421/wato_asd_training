#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include "costmap_node.hpp"
#include "costmap_core.hpp"

CostmapNode::CostmapNode()
: Node("costmap_node"),
  costmap_(this->get_logger()),
  resolution_(0.1),
  width_(1000),
  height_(1000),
  origin_x_(-50),
  origin_y_(-50),
  inflation_radius_(1),
  max_cost_(100)
{
    // Create subscription with correct type
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
    
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
    
    
    this->declare_parameter("inflation_radius", inflation_radius_);
    this->declare_parameter("max_cost", max_cost_);
    
    inflation_radius_ = this->get_parameter("inflation_radius").as_double();
    max_cost_ = this->get_parameter("max_cost").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Costmap node initialized");
}

void CostmapNode::initializeCostmap()
{
    costmap_.initialize(width_, height_, 0);
    
    RCLCPP_INFO(this->get_logger(), 
        "Initialized costmap with size %dx%d (resolution: %.2f m/cell)",
        width_, height_, resolution_);
}

void CostmapNode::convertToGrid(float range, float angle, int& x_grid, int& y_grid) const
{
    float x = range * cos(angle);
    float y = range * sin(angle);
    
    x_grid = static_cast<int>((x - origin_x_) / resolution_);
    y_grid = static_cast<int>((y - origin_y_) / resolution_);
}

void CostmapNode::markObstacle(int x, int y)
{
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
      costmap_.at(x, y) = 100;  // Use bounds-checked access
    }
}

void CostmapNode::inflateObstacles()
{
    auto current_grid = costmap_.getGrid();
    auto new_costs = current_grid;  // Make a copy
    
    int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);
    
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (current_grid[y][x] == 100) {
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            if (dx == 0 && dy == 0) continue;
                            
                            float distance = resolution_ * sqrt(dx*dx + dy*dy);
                            if (distance <= inflation_radius_) {
                                int cost = static_cast<int>(max_cost_ * (1.0 - distance/inflation_radius_));
                                if (cost > new_costs[ny][nx] && new_costs[ny][nx] < 100) {
                                    new_costs[ny][nx] = cost;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    costmap_.setGrid(new_costs);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    initializeCostmap();
    
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float angle = scan->angle_min + i * scan->angle_increment;
        float range = scan->ranges[i];
        
        if (range < scan->range_max && range > scan->range_min) {
            int x, y;
            convertToGrid(range, angle, x, y);
            markObstacle(x, y);
        }
    }
    
    inflateObstacles();
    //RCLCPP_INFO(this->get_logger(), "blaj blaj dfs");
    publishCostmap(scan);
}

void CostmapNode::publishCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    nav_msgs::msg::OccupancyGrid costmapmsg;
    const auto& grid = costmap_.getGrid();
    
    costmapmsg.header = scan->header;
 
    
    costmapmsg.info.resolution = resolution_;
    costmapmsg.info.width = width_;
    costmapmsg.info.height = height_;
    costmapmsg.info.origin.position.x = origin_x_;
    costmapmsg.info.origin.position.y = origin_y_;
    costmapmsg.info.origin.orientation.w = 1.0;
    
    costmapmsg.data.resize(width_ * height_);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
          costmapmsg.data[y * width_ + x] = grid[y][x];
        }
    }
    
    costmap_pub_->publish(costmapmsg);
    // nav_msgs::msg::OccupancyGrid test_msg;
    // test_msg.data = {0};  // Simple test message
    // costmap_pub_->publish(test_msg);
    // RCLCPP_INFO(this->get_logger(), "Published test message");

  }

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}