#include "costmap_core.hpp"
#include <stdexcept>

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
    : width_(0), height_(0), logger_(logger) {}

void CostmapCore::initialize(unsigned long width, unsigned long height, int8_t value) {
    width_ = width;
    height_ = height;
    grid_.resize(height);
    for (auto& row : grid_) {
        row.assign(width, value);
    }
}

int8_t& CostmapCore::at(unsigned long x, unsigned long y) {
    if (x >= width_ || y >= height_) {
        throw std::out_of_range("Costmap coordinates out of range");
    }
    return grid_[y][x];
}

const int8_t& CostmapCore::at(unsigned long x, unsigned long y) const {
    if (x >= width_ || y >= height_) {
        throw std::out_of_range("Costmap coordinates out of range");
    }
    return grid_[y][x];
}

void CostmapCore::setGrid(const std::vector<std::vector<int8_t>>& grid) {
    if (!grid.empty()) {
        width_ = grid[0].size();
        height_ = grid.size();
    } else {
        width_ = 0;
        height_ = 0;
    }
    grid_ = grid;
}

} // namespace robot