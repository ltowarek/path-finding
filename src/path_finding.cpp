#include "path_finding.h"

int pathfinding::PathFinding::FindPath(const int start_x, const int start_y,
                                       const int target_x, const int target_y,
                                       const unsigned char *map, const int map_width, const int map_height,
                                       int *output, const int output_size) const {
  auto start = Point{start_x, start_y};
  auto target = Point{target_x, target_y};

  typedef std::pair<Point, int> PointWithPriority;
  auto frontier = std::priority_queue<PointWithPriority, std::vector<PointWithPriority>, std::greater<PointWithPriority>>();
  frontier.emplace(start, Heuristic(start, target));

  auto came_from = std::unordered_map<Point, Point, PointHasher>();
  auto cost_so_far = std::unordered_map<Point, int, PointHasher>();

  came_from[start] = start;
  cost_so_far[start] = 0;

  while(!frontier.empty()) {
    auto current = frontier.top().first;
    frontier.pop();

    if (current == target) {
      auto path = ReconstructPath(start, target, came_from);

      if (path.size() > output_size) {
        return -2;
      }

      for (int i = 0; i < path.size(); ++i) {
        output[i] = path[i].y * map_width + path[i].x;
      }

      return static_cast<int>(path.size());
    }

    for (auto next : Neighbours(map, map_width, map_height, current)) {
      int new_cost = cost_so_far[current] + Cost(current, next);
      if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        int priority = new_cost + Heuristic(next, target);
        frontier.emplace(next, priority);
        came_from[next] = current;
      }
    }
  }

  return -1;
}
