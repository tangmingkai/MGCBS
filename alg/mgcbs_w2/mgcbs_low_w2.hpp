#pragma once
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include "env/environment.hpp"
#include "base/location.hpp"
#include "base/ts_state.hpp"
#include <boost/heap/d_ary_heap.hpp>

namespace mg_cbs_w2 {
class MGCBSLow {
 public:
  explicit MGCBSLow(const Environment &env) : env_(env) {}
  int Search(const TSState &start_state, const TISState &goal,
             const Constraints &cons,
             const std::vector<std::vector<int>> &distance_matrix,
             std::vector<TSState> *path) {
    if (cons.IsRestricted(start_state)) {
      return Interval::oo;
    }
    if (start_state.GetLocation() == goal.GetLocation() &&
        goal.interval.IsIn(start_state.time)) {
      path->clear();
      return start_state.time;
    }
    OpenSet open_set;
    StateToHeap state_to_heap;
    ClosedSet closed_set;
    ComeFrom come_from;
    auto handle = open_set.push(
        LowLevelNode(start_state, start_state.time,
                   std::max(distance_matrix[start_state.x][start_state.y],
                            goal.interval.l - start_state.time)));
    state_to_heap[start_state] = handle;
    std::vector<TSState> neighbors;
    neighbors.reserve(10);
    while (!open_set.empty()) {
      LowLevelNode current = open_set.top();
      if ((goal.interval.IsIn(current.state.time)) &&
          (current.state.GetLocation() == goal.GetLocation())) {
        path->clear();
        path->push_back(current.state);
        auto iter = come_from.find(current.state);
        while (iter != come_from.end()) {
          path->push_back(iter->second);
          iter = come_from.find(iter->second);
        }
        path->pop_back();
        std::reverse(path->begin(), path->end());
        return path->back().time;
      }

      open_set.pop();
      state_to_heap.erase(current.state);
      closed_set.insert(current.state);
      GetVaildNeighbors(current.state, cons, &neighbors);
      for (const TSState& neighbor : neighbors) {
        if (closed_set.find(neighbor) == closed_set.end()) {
          int g = current.g + 1;
          auto iter = state_to_heap.find(neighbor);
          if (iter == state_to_heap.end()) {  // Discover a new node
            int h = std::max(distance_matrix[neighbor.x][neighbor.y],
                             goal.interval.l - g);
            int f = g + h;
            auto handle = open_set.push(LowLevelNode(neighbor, g, f));
            state_to_heap[neighbor] = handle;
          } else {
            auto handle = iter->second;
            if (g >= (*handle).g) {
              continue;
            }

            int delta = (*handle).g - g;
            (*handle).g = g;
            (*handle).f -= delta;
            open_set.update(handle);
          }
          come_from[neighbor] = current.state;
        }
      }
    }
    return Interval::oo;
  }
  struct LowLevelNode {
    LowLevelNode(const TSState &state, const int &g, const int &f)
        : state(state), g(g), f(f) {}

    TSState state;
    int index;
    int g, f;

    bool operator<(const LowLevelNode &n) const {
      if (f != n.f) return f > n.f;
      return g < n.g;
    }

    friend std::ostream &operator<<(std::ostream &os, const LowLevelNode &c) {
      os << c.state << " " << c.index << " " << c.f << "=" << c.g << "+"
         << (c.f - c.g) << std::endl;
      return os;
    }
  };
  typedef boost::heap::d_ary_heap<LowLevelNode, boost::heap::arity<2>,
                                  boost::heap::mutable_<true>>
      OpenSet;
  typedef OpenSet::handle_type HeapHandle_t;

  typedef std::unordered_map<TSState, HeapHandle_t> StateToHeap;
  typedef std::unordered_set<TSState> ClosedSet;
  typedef std::unordered_map<TSState, TSState> ComeFrom;

  void GetVaildNeighbors(const TSState &s1, const Constraints &cons,
                         std::vector<TSState> *vaild_neighbors) {
    vaild_neighbors->clear();
    std::vector<Neighbor> neighbors;
    env_.GetNeighbors(s1.GetLocation(), &neighbors);
    for (const auto &neighbor : neighbors) {
      TSState s2(s1.time + 1, neighbor.location.x, neighbor.location.y);
      if (cons.IsRestricted(s1, s2)) continue;
      vaild_neighbors->emplace_back(s2);
    }
  }
  const Environment &env_;
};
};  // namespace mg_cbs_w2
