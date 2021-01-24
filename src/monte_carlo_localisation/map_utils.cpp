#include "monte_carlo_localisation/map_utils.hpp"
#include <cmath>

int MapUtils::mapCoordToIndex(const double &x, const double &y, const uint32_t &map_width)
{
    return floor(x + y*map_width);
}
std::pair<int, int> MapUtils::worldToMapCoord(const double &x, const double &y, const nav_msgs::MapMetaData &map_info)
{
    int map_x = floor((x - map_info.origin.position.x) / map_info.resolution + 0.5);
    int map_y = floor((y - map_info.origin.position.y) / map_info.resolution + 0.5);
    return std::make_pair(map_x, map_y);
}
std::pair<uint32_t, uint32_t> MapUtils::mapIndexToCoord(const int &index, const uint32_t &map_width)
{
    uint32_t x = index % map_width;
    uint32_t y = (index - x)/map_width;
    return std::make_pair(x, y);
}
double MapUtils::distanceBetweenTwoPoints(const int &x1, const int &y1,
                                             const int &x2, const int &y2)
{
    return hypot(x1-x2, y1-y2);
}