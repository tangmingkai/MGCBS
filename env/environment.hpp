#pragma once

#include <cstdlib>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <map>
#include <utility>
#include <algorithm>

#include "../base/action.hpp"
#include "../base/conflict.hpp"
#include "../base/contraints.hpp"
#include "../base/edge_constraint.hpp"
#include "../base/location.hpp"
#include "../base/neighbor.hpp"
#include "../base/ocp.hpp"
#include "../base/plan_result.hpp"
#include "../base/tis_state.hpp"
#include "../base/vertex_constraint.hpp"

class Environment {
 public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles)
      : dimx_(dimx), dimy_(dimy), obstacles_(std::move(obstacles)) {
    reverse_neighbor_dirs_.emplace_back(NeighborDir(0, 0, Action::Wait, 1));
    reverse_neighbor_dirs_.emplace_back(NeighborDir(1, 0, Action::Left, 1));
    reverse_neighbor_dirs_.emplace_back(NeighborDir(0, 1, Action::Down, 1));
    reverse_neighbor_dirs_.emplace_back(NeighborDir(-1, 0, Action::Right, 1));
    reverse_neighbor_dirs_.emplace_back(NeighborDir(0, -1, Action::Up, 1));

    neighbor_dirs_.emplace_back(NeighborDir(0, 0, Action::Wait, 1));
    neighbor_dirs_.emplace_back(NeighborDir(-1, 0, Action::Left, 1));
    neighbor_dirs_.emplace_back(NeighborDir(0, -1, Action::Down, 1));
    neighbor_dirs_.emplace_back(NeighborDir(1, 0, Action::Right, 1));
    neighbor_dirs_.emplace_back(NeighborDir(0, 1, Action::Up, 1));
  }

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  int GetID(int x, int y) const  { return x * dimy_ + y; }
  int GetX(int id) const { return id / dimy_; }
  int GetY(int id) const { return id % dimy_; }
  void GetReverseNeighbors(const Location& s,
                    std::vector<Neighbor>* neighbors) const {
    neighbors->clear();
    for (auto& dir : reverse_neighbor_dirs_) {
        Location n(s.x + dir.dx, s.y + dir.dy);
        if (LocationVaild(n))
          neighbors->emplace_back(
              Neighbor(n, dir.action, dir.cost));
      }
  }
  void GetNeighbors(const Location& s,
                    std::vector<Neighbor>* neighbors) const {
    neighbors->clear();
    for (auto& dir : neighbor_dirs_) {
        Location n(s.x + dir.dx, s.y + dir.dy);
        if (LocationVaild(n))
          neighbors->emplace_back(
              Neighbor(n, dir.action, dir.cost));
      }
  }

  bool GetFirstConflict(const std::vector<PlanResultPtr>& solutions,
                        Conflict* result) const {
    std::unordered_map<VertexOcp, uint> vertex_ocp_map;
    std::unordered_map<EdgeOcp, uint> edge_ocp_map;
    uint max_time = 0;
    for (uint solution_id = 0; solution_id < solutions.size(); solution_id++) {
      max_time = std::max(
          max_time, static_cast<uint>(solutions[solution_id]->states.size()));
    }
    for (uint time = 0; time < max_time; time++) {
      for (uint solution_id = 0; solution_id < solutions.size();
           solution_id++) {
        auto& states = solutions[solution_id]->states;
        auto now_state =
            states[std::min(time, static_cast<uint>(states.size()) - 1)];
        now_state.time = time;
        VertexOcp vertex_ocp =
            VertexOcp(now_state.time, now_state.x, now_state.y);
        auto iter = vertex_ocp_map.find(vertex_ocp);
        if (iter != vertex_ocp_map.end()) {
          result->time = time;
          result->agent1 = iter->second;
          result->agent2 = solution_id;
          result->type = Conflict::Vertex;
          result->x1 = now_state.x;
          result->y1 = now_state.y;
          return true;
        }
        vertex_ocp_map[vertex_ocp] = solution_id;

        if (time + 1 < static_cast<uint>(states.size())) {
          auto& next_state = states[time + 1];
          auto edge_ocp = EdgeOcp(now_state.time, now_state.x, now_state.y,
                                  next_state.x, next_state.y);
          auto iter = edge_ocp_map.find(edge_ocp);
          if (iter != edge_ocp_map.end()) {
            result->time = now_state.time;
            result->agent1 = solution_id;
            result->agent2 = iter->second;
            result->type = Conflict::Edge;
            result->x1 = now_state.x;
            result->y1 = now_state.y;
            result->x2 = next_state.x;
            result->y2 = next_state.y;
            return true;
          }
          auto tmp_edge_ocp = EdgeOcp(now_state.time, next_state.x,
                                      next_state.y, now_state.x, now_state.y);
          edge_ocp_map[tmp_edge_ocp] = solution_id;
        }
      }
    }
    return false;
  }

  bool GetFirstConflict(
    const std::vector<PlanResult>& solutions, Conflict* result) const  {
    std::unordered_map<VertexOcp, uint> vertex_ocp_map;
    std::unordered_map<EdgeOcp, uint> edge_ocp_map;
    uint max_time = 0;
    for (uint solution_id = 0; solution_id < solutions.size(); solution_id++) {
      max_time = std::max(
          max_time, static_cast<uint>(solutions[solution_id].states.size()));
    }
    for (uint time = 0; time < max_time; time++) {
      for (uint solution_id = 0; solution_id < solutions.size();
           solution_id++) {
        auto& states = solutions[solution_id].states;
        if (time >=states.size())
          continue;
        auto& now_state = states[time];
        VertexOcp vertex_ocp =
            VertexOcp(now_state.time, now_state.x, now_state.y);
        auto iter = vertex_ocp_map.find(vertex_ocp);
        if (iter != vertex_ocp_map.end()) {
          result->time = time;
          result->agent1 = iter->second;
          result->agent2 = solution_id;
          result->type = Conflict::Vertex;
          result->x1 = now_state.x;
          result->y1 = now_state.y;
          return true;
        }
        vertex_ocp_map[vertex_ocp] = solution_id;

        if (time + 1 < static_cast<uint>(states.size())) {
          auto& next_state = states[time + 1];
          auto edge_ocp = EdgeOcp(now_state.time, now_state.x, now_state.y,
                                  next_state.x, next_state.y);
          auto iter = edge_ocp_map.find(edge_ocp);
          if (iter != edge_ocp_map.end()) {
            result->time = now_state.time;
            result->agent1 = solution_id;
            result->agent2 = iter->second;
            result->type = Conflict::Edge;
            result->x1 = now_state.x;
            result->y1 = now_state.y;
            result->x2 = next_state.x;
            result->y2 = next_state.y;
            return true;
          }
          auto tmp_edge_ocp = EdgeOcp(now_state.time, next_state.x,
                                      next_state.y, now_state.x, now_state.y);
          edge_ocp_map[tmp_edge_ocp] = solution_id;
        }
      }
    }
    return false;
  }

  void CreateConstraintsFromConflict(
      const Conflict& conflict,
      std::map<size_t, Constraints>& constraints) const {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.AddVertexConstraint(conflict.time, conflict.x1, conflict.y1);
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.AddEdgeConstraint(conflict.time, conflict.x1, conflict.y1, conflict.x2,
                           conflict.y2);
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.AddEdgeConstraint(conflict.time, conflict.x2, conflict.y2, conflict.x1,
                           conflict.y1);
      constraints[conflict.agent2] = c2;
    }
  }

  bool LocationVaild(const Location& l) const {
    return l.x >= 0 && l.x < dimx_ && l.y >= 0 && l.y < dimy_ &&
           obstacles_.find(l) == obstacles_.end();
  }

  int dimx() const {return dimx_;}
  int dimy() const {return dimy_;}

 private:
  int dimx_;
  int dimy_;
  std::unordered_set<Location> obstacles_;
  std::vector<NeighborDir> reverse_neighbor_dirs_;
  std::vector<NeighborDir> neighbor_dirs_;
};
