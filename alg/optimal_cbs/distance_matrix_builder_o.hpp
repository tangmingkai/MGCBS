#pragma once
#include <vector>
#include "base/agent.hpp"
#include "base/interval.hpp"
#include "env/environment.hpp"
#include <boost/heap/d_ary_heap.hpp>
namespace optimal_cbs {
class DistanceMatrixBuilder {
 public:
  struct DNode {
    DNode() {}
    DNode(const Location &loc, int cost) : loc(loc), cost(cost) {}
    Location loc;
    int cost;
    bool operator<(const DNode &n) const { return cost > n.cost; }
  };
  explicit DistanceMatrixBuilder(const Environment &environment)
      : env_(environment) {}
  void BuildDistanceMatrix(
    const Agent &agent,
    std::vector<std::vector<std::vector<int>>> *full_distance_matrix,
    std::vector<std::vector<int>> *target_distance_matrix) {
    (*full_distance_matrix) = std::vector<std::vector<std::vector<int>>>(
        agent.targets.size(),
        std::vector<std::vector<int>>(
            env_.dimx(), std::vector<int>(env_.dimy(), Interval::oo)));
    (*target_distance_matrix) = std::vector<std::vector<int>>(
        agent.targets.size(),
        std::vector<int>(agent.targets.size(), Interval::oo));
    for (int i = 1; i < static_cast<int>(agent.targets.size()); i++) {
      std::vector<std::vector<int>> full_map_distance;
      FullMapDistanceToOneGoal(agent.targets[i], &full_map_distance);
      (*full_distance_matrix)[i] = full_map_distance;
      for (int j = 0; j < static_cast<int>(agent.targets.size()); j++)
        (*target_distance_matrix)[j][i] =
            full_map_distance[agent.targets[j].x][agent.targets[j].y];
    }
  }
  int GetID(int x, int y) { return x * env_.dimy() + y; }
  int GetX(int id) { return id / env_.dimy(); }
  int GetY(int id) { return id % env_.dimy(); }

  void FullMapDistanceToOneGoal(
      const Location &goal, std::vector<std::vector<int>> *full_map_distance) {
    boost::heap::d_ary_heap<DNode, boost::heap::arity<2>,
                            boost::heap::mutable_<true>>
        open;
    std::vector<std::vector<
        boost::heap::d_ary_heap<DNode, boost::heap::arity<2>,
                                boost::heap::mutable_<true>>::handle_type>>
        handle(env_.dimx(),
               std::vector<boost::heap::d_ary_heap<
                   DNode, boost::heap::arity<2>,
                   boost::heap::mutable_<true>>::handle_type>(env_.dimy()));
    *full_map_distance = std::vector<std::vector<int>>(
        env_.dimx(), std::vector<int>(env_.dimy(), Interval::oo));
    (*full_map_distance)[goal.x][goal.y] = 0;
    for (int i = 0; i < env_.dimx(); i++) {
      for (int j = 0; j< env_.dimy(); j++) {
        DNode node(Location(i, j), (*full_map_distance)[i][j]);
        handle[i][j] = open.push(node);
      }
    }
    while (!open.empty()) {
      auto n = open.top();
      open.pop();
      int current_c = n.cost;
      auto current_location = n.loc;
      std::vector<Neighbor> neighbors;
      env_.GetReverseNeighbors(current_location, &neighbors);
      for (uint j = 0; j < neighbors.size(); j++) {
        auto &neighbour = neighbors[j];
        if (current_location == neighbour.location) continue;
        if ((*full_map_distance)[neighbour.location.x][neighbour.location.y] >
            current_c + 1) {
          (*full_map_distance)[neighbour.location.x][neighbour.location.y] =
              current_c + 1;
          (*handle[neighbour.location.x][neighbour.location.y]).cost =
              current_c + 1;
          open.update(handle[neighbour.location.x][neighbour.location.y]);
        }
      }
    }
  }

 private:
  const Environment &env_;
};
}  // namespace optimal_cbs
