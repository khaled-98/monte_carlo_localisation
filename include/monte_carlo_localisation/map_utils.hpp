#ifndef B8A400C8_F58A_4931_8E05_BBB95F2BF8F6
#define B8A400C8_F58A_4931_8E05_BBB95F2BF8F6

#include "nav_msgs/OccupancyGrid.h"
#include <cstdint>
#include <utility>

class MapUtils
{
public:
    static int mapCoordToIndex(const double &x, const double &y, const uint32_t &map_width);
    static std::pair<int, int> worldToMapCoord(const double &x, const double &y, const nav_msgs::MapMetaData &map_info);
    static std::pair<uint32_t, uint32_t> mapIndexToCoord(const int &index, const uint32_t &map_width);
    static double distanceBetweenTwoPoints(const int &x1, const int &y1,
                                              const int &x2, const int &y2);
};

#endif /* B8A400C8_F58A_4931_8E05_BBB95F2BF8F6 */
