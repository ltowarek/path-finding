#include "path_finding.h"

int pathfinding::PathFinding::FindPath(const int start_x, const int start_y,
                                       const int target_x, const int target_y,
                                       const unsigned char *map, const int map_width, const int map_height,
                                       int *output, const int output_size) const {
  auto start = Point{start_x, start_y};
  auto target = Point{target_x, target_y};

  typedef std::pair<int, Point> PointWithPriority;
  auto frontier =
      std::priority_queue<PointWithPriority, std::vector<PointWithPriority>, std::greater<PointWithPriority>>();
  frontier.emplace(Heuristic(start, target), start);

  auto came_from = std::unordered_map<Point, Point, PointHasher>();
  auto cost_so_far = std::unordered_map<Point, int, PointHasher>();

  came_from[start] = start;
  cost_so_far[start] = 0;

  while (!frontier.empty()) {
    auto current = frontier.top().second;
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
        frontier.emplace(priority, next);
        came_from[next] = current;
      }
    }
  }

  return -1;
}

std::vector<pathfinding::Point> pathfinding::PathFinding::Neighbours(const unsigned char *map,
                                                                     const int map_width,
                                                                     const int map_height,
                                                                     const Point &point) const {
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

int pathfinding::PathFinding::Cost(const Point &p1, const Point &p2) const {
  return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
}

int pathfinding::PathFinding::Heuristic(const Point &p1, const Point &p2) const {
  return Cost(p1, p2);
}

std::vector<pathfinding::Point> pathfinding::PathFinding::ReconstructPath(const Point &start,
                                                                          const Point &target,
                                                                          std::unordered_map<Point,
                                                                                             Point,
                                                                                             PointHasher> &came_from) const {
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
