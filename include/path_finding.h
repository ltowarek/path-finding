#ifndef PATH_FINDING_PATH_FINDING_H
#define PATH_FINDING_PATH_FINDING_H

namespace pathfinding {
class PathFinding {
 public:
  int FindPath(const int start_x, const int start_y,
               const int target_x, const int target_y,
               const unsigned char *map, const int map_width, const int map_height,
               int *output, const int output_size) const;
 private:
};
}  // pathfinding

#endif //PATH_FINDING_PATH_FINDING_H