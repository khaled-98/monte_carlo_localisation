#ifndef B8A400C8_F58A_4931_8E05_BBB95F2BF8F6
#define B8A400C8_F58A_4931_8E05_BBB95F2BF8F6

#include <cstdint>
#include <utility>

class MapUtils
{
public:
    static int coordinates_to_index(const uint32_t &x, const uint32_t &y, const uint32_t &map_width);
    static std::pair<uint32_t, uint32_t> index_to_coordinates(const int &index, const uint32_t &map_width);
    static double distance_between_two_points(const uint32_t &x1, const uint32_t &y1,
                                              const uint32_t &x2, const uint32_t &y2);
};

#endif /* B8A400C8_F58A_4931_8E05_BBB95F2BF8F6 */
