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

class PathFinding {
 public:
  int FindPath(const int start_x, const int start_y,
               const int target_x, const int target_y,
               const unsigned char *map, const int map_width, const int map_height,
               int *output, const int output_size) const;
 private:
  std::vector<Point> Neighbours(const unsigned char *map, const int map_width, const int map_height, const Point &point) const {
    auto neighbours = std::vector<Point>();
    if ((point.x + 1 < map_width) && (map[point.y * map_width + point.x + 1] == 1)) {
      auto neighbour = point;
      neighbour.x += 1;
      neighbours.push_back(neighbour);
    }
    if ((point.x - 1 >= 0) && (map[point.y * map_width + point.x - 1] == 1)) {
      auto neighbour = point;
      neighbour.x -= 1;
      neighbours.push_back(neighbour);
    }
    if ((point.y + 1 < map_height) && (map[(point.y + 1) * map_width + point.x] == 1)) {
      auto neighbour = point;
      neighbour.y += 1;
      neighbours.push_back(neighbour);
    }
    if ((point.y - 1 >= 0) && (map[(point.y - 1) * map_width + point.x] == 1)) {
      auto neighbour = point;
      neighbour.y -= 1;
      neighbours.push_back(neighbour);
    }
    return neighbours;
  }

  int Cost(const Point &p1, const Point &p2) const {
    return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
  }

  int Heuristic(const Point &p1, const Point &p2) const {
    return Cost(p1, p2);
  }

  std::vector<Point> ReconstructPath(const Point &start, const Point &target, std::unordered_map<Point, Point, PointHasher> &came_from) const {
    std::vector<Point> path;
    Point current = target;
    path.push_back(current);
    while (current != start) {
      current = came_from[current];
      path.push_back(current);
    }
    path.pop_back();
    std::reverse(path.begin(), path.end());
    return path;
  }
};
}  // pathfinding

#endif //PATH_FINDING_PATH_FINDING_H