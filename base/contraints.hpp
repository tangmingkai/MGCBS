#pragma once
#include <list>
#include <unordered_set>
#include <utility>
#include <vector>
#include <unordered_map>
#include <memory>
#include "base/edge_constraint.hpp"
#include "base/vertex_constraint.hpp"
struct Constraints {
  Constraints() { Clear(); }
  std::unordered_map<Location, std::list<VertexConstraint>> vertex_constraints;
  std::unordered_map<std::pair<Location, Location>, std::vector<EdgeConstraint>>
      edge_constraints;
  bool IsRestricted(const TSState &state) const {
    auto iter = vertex_constraints.find(state.GetLocation());
    if (iter == vertex_constraints.end())
      return false;
    for (auto& t : iter->second) {
      if (t.time == state.time) {
        return true;
      }
    }
    return false;
  }
  bool IsRestricted(const TSState &state, const TSState &next_state) const {
    if (IsRestricted(next_state))
      return true;
    auto iter = edge_constraints.find(
        std::make_pair(state.GetLocation(), next_state.GetLocation()));
    if (iter == edge_constraints.end()) return false;
    for (auto& t : iter->second) {
      if (t.time == state.time) {
        return true;
      }
    }
    return false;
  }
  void Clear() {
    vertex_constraints.clear();
    edge_constraints.clear();
  }
  void AddEdgeConstraint(int t, int x1, int y1, int x2, int y2) {
    Location a1(x1, y1);
    Location a2(x2, y2);
    auto &ec = edge_constraints[std::make_pair(a1, a2)];
    if (ec.empty() || t >= ec.back().time) {
      ec.insert(ec.end(), EdgeConstraint(t, x1, y1, x2, y2));
    } else {
      for (auto iter = ec.begin(); iter != ec.end(); ++iter) {
          if (t <= iter->time) {
              ec.insert(iter, EdgeConstraint(t, x1, y1, x2, y2));
              break;
          }
      }
    }
  }
  void AddVertexConstraint(int t, int x1, int y1) {
    Location a1(x1, y1);
    auto &vc = vertex_constraints[a1];
    if (vc.empty() || t >= vc.back().time) {
      vc.insert(vc.end(), VertexConstraint(t, x1, y1));
    } else {
      for (auto iter = vc.begin(); iter != vc.end(); ++iter) {
          if (t <= iter->time) {
              vc.insert(iter, VertexConstraint(t, x1, y1));
              break;
          }
      }
    }
  }
  void Add(const Constraints& other) {
    for (const auto& iter1 : other.vertex_constraints) {
      for (const auto& iter2 : iter1.second) {
        AddVertexConstraint(iter2.time, iter2.x, iter2.y);
      }
    }
    for (const auto& iter1 : other.edge_constraints) {
      for (const auto& iter2 : iter1.second) {
        AddEdgeConstraint(iter2.time, iter2.x1, iter2.y1, iter2.x2, iter2.y2);
      }
    }
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& iter1 : c.vertex_constraints) {
      for (const auto& iter2 : iter1.second) {
        os << iter2 << std::endl;
      }
    }
    for (const auto& iter1 : c.edge_constraints) {
      for (const auto& iter2 : iter1.second) {
        os << iter2 << std::endl;
      }
    }
    return os;
  }
};

typedef std::shared_ptr<Constraints> ConstraintsPtr;
