#ifndef PATH_FINDING_PATH_FINDING_H
#define PATH_FINDING_PATH_FINDING_H

#include <algorithm>
#include <queue>
#include <unordered_map>

namespace pathfinding {
struct Point {
  int x;
  int y;

  bool operator<(const Point &rhs) const {
    return (x < rhs.x) && (y < rhs.y);
  }

  bool operator>(const Point &rhs) const {
    return (x > rhs.x) && (y > rhs.y);
  }

  bool operator==(const Point &rhs) const {
    return (x == rhs.x) && (y == rhs.y);
  }

  bool operator!=(const Point &rhs) const {
    return !(*this == rhs);
  }
};

struct PointHasher {
  std::size_t operator()(const Point &p) const {
    return std::hash<int>()(p.x) ^ std::hash<int>()(p.y);
  }
};

class PathNotFoundException : public std::exception {
  const char *what() const throw() {
    return "Path not found";
  }
};

class PathFinding {
 public:
  int FindPath(const int start_x, const int start_y,
               const int target_x, const int target_y,
               const unsigned char *map, const int map_width, const int map_height,
               int *output, const int output_size) const;
 private:
  std::unordered_map<Point, Point, PointHasher> Search(const Point &start,
                                                       const Point &target,
                                                       const unsigned char *map,
                                                       const int map_width,
                                                       const int map_height) const;
  std::vector<Point> Neighbours(const unsigned char *map,
                                const int map_width,
                                const int map_height,
                                const Point &point) const;
  bool HasNorthernNeighbour(const Point &point,
                            const int map_width,
                            const int map_height,
                            const unsigned char *map) const;
  bool HasSouthernNeighbour(const Point &point,
                            const int map_width,
                            const int map_height,
                            const unsigned char *map) const;
  bool HasEasternNeighbour(const Point &point,
                           const int map_width,
                           const int map_height,
                           const unsigned char *map) const;
  bool HasWesternNeighbour(const Point &point,
                           const int map_width,
                           const int map_height,
                           const unsigned char *map) const;
  int Cost(const Point &p1, const Point &p2) const;
  int Heuristic(const Point &p1, const Point &p2) const;
  std::vector<Point> ReconstructPath(const Point &start,
                                     const Point &target,
                                     std::unordered_map<Point, Point, PointHasher> &came_from) const;
  void FillOutput(const std::vector<Point> path, const int map_width, const int output_size, int *output) const;
  int GetPointId(const Point &point, const int map_width) const;
};
}  // pathfinding

#endif //PATH_FINDING_PATH_FINDING_H