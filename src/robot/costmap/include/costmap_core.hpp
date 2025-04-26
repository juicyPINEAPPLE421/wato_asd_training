#ifndef COSTMAP_CORE_HPP
#define COSTMAP_CORE_HPP

#include <rclcpp/logger.hpp>
#include <vector>
#include <cstdint>

namespace robot {

class CostmapCore {
public:
    explicit CostmapCore(const rclcpp::Logger& logger);
    
    // Grid initialization
    void initialize(unsigned long width, unsigned long height, int8_t value = 0);
    
    // Grid access
    int8_t& at(unsigned long x, unsigned long y);
    const int8_t& at(unsigned long x, unsigned long y) const;
    
    // Grid information
    unsigned long width() const { return width_; }
    unsigned long height() const { return height_; }
    
    // Grid data access
    const std::vector<std::vector<int8_t>>& getGrid() const { return grid_; }
    void setGrid(const std::vector<std::vector<int8_t>>& grid);

private:
    std::vector<std::vector<int8_t>> grid_;
    unsigned long width_;
    unsigned long height_;
    rclcpp::Logger logger_;
};

} // namespace robot

#endif // COSTMAP_CORE_HPP