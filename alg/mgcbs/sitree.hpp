#pragma once
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>
#include <utility>
#include <algorithm>
#include "base/location.hpp"
#include "base/neighbor.hpp"
#include "base/tis_state.hpp"
#include "env/environment.hpp"
#include <boost/heap/d_ary_heap.hpp>
namespace mg_cbs {
// The TIS Tree in the paper
class SITree{
 public:
  struct SINode;
  typedef boost::heap::d_ary_heap<SINode, boost::heap::arity<2>,
                                           boost::heap::mutable_<true>>
      OpenSet;
  typedef OpenSet::handle_type HeapHandle_t;
  typedef std::unordered_map<Location,
                             std::list<SINode>>
      SINodePool;
  SINodePool si_node_pool_;
  struct SINode {
    TISState state;
    int cost;
    TISState come_from;
    HeapHandle_t handle;
    bool operator<(const SINode &n) const {
      return cost > n.cost;
    }
  };

  explicit SITree(const Environment &env) : env_(env) {}
  void Build(const TISState &goal_state, const Constraints &cons) {
    goal_state_ = goal_state;
    OpenSet open_set;
    si_node_pool_.clear();
    {
      std::list<SINode> *begin_nodes;
      CheckAndNew(goal_state.GetLocation(), cons, &si_node_pool_, &begin_nodes,
                  &open_set);
      SINode *begin_node;
      if (!FindSINode(goal_state, &begin_node)) {
        std::cout << "Cannot find start node" << std::endl;
        exit(1);
      }
      begin_node->cost = 0;
      auto handle = open_set.push(*begin_node);
      begin_node->handle = handle;
    }
    std::vector<Neighbor> neighbors;
    std::vector<Interval> split_intervals;
    while (!open_set.empty()) {
      const auto current = open_set.top();
      if (current.cost == Interval::oo) return;
      const auto current_loc = current.state.GetLocation();
      open_set.pop();
      env_.GetReverseNeighbors(current_loc, &neighbors);
      for (auto &neighbor : neighbors) {
        auto next_location = neighbor.location;
        std::list<SINode> *node_list;
        CheckAndNew(next_location, cons, &si_node_pool_, &node_list, &open_set);
        SplitInterval(current.state.interval - 1, cons, current_loc,
                      next_location, &split_intervals);
        Update(split_intervals, current.cost + neighbor.cost, current.state,
               node_list, &open_set);
      }
    }
  }
  int GetCost(const TSState &ts_state) const {
    SINode node;
    if (ts_state.time > goal_state_.interval.r)
      return Interval::oo;
    if (FindSINode(ts_state, &node)) {
      return node.cost;
    }
    return Interval::oo;
  }

  bool FindSINode(const TSState &ts_state, SINode *si_node) const {
    auto iter = si_node_pool_.find(ts_state.GetLocation());
    if (iter == si_node_pool_.end()) return false;
    for (auto &iter2 : iter->second) {
      if (iter2.state.interval.IsIn(ts_state.time)) {
        *si_node = iter2;
        return true;
      }
    }
    return false;
  }

  bool FindSINode(const TISState &tis_state, SINode *si_node) const {
    auto iter = si_node_pool_.find(tis_state.GetLocation());
    if (iter == si_node_pool_.end()) return false;
    for (auto &iter2 : iter->second) {
      if (iter2.state == tis_state) {
        *si_node = iter2;
        return true;
      }
    }
    return false;
  }

  bool FindSINode(const TISState &tis_state, SINode **si_node) {
    auto iter = si_node_pool_.find(tis_state.GetLocation());
    if (iter == si_node_pool_.end()) return false;
    for (auto &iter2 : iter->second) {
      if (iter2.state == tis_state) {
        *si_node = (&iter2);
        return true;
      }
    }
    return false;
  }

  bool AppendPath(const TSState &start_ts_state,
                  PlanResult *plan_result) const {
    SINode si_node;
    if (!FindSINode(start_ts_state, &si_node)) {
      return false;
    }
    int now_time = start_ts_state.time;
    while (true) {
      auto &next_tis_state = si_node.come_from;
      if (next_tis_state.interval.r < 0) break;
      now_time++;
      plan_result->states.emplace_back(
          TSState(now_time, next_tis_state.x, next_tis_state.y));
      if (!FindSINode(next_tis_state, &si_node))
        break;
    }
    if (plan_result->states.back().GetLocation() != goal_state_.GetLocation()) {
      std::cout << "Error. path doesn't reach goal." << std::endl;
      exit(0);
    }
    if (!goal_state_.interval.IsIn(plan_result->states.back().time)) {
      std::cout << "Error. path doesn't reach time interval." << std::endl;
      exit(0);
    }
    return true;
  }

 private:
  void CheckAndNew(const Location &location,
                   const Constraints &constraints,
                   SINodePool *node_pool,
                   std::list<SINode> **node_list,
                   OpenSet *open_set) {
    auto np_iter =  node_pool->find(location);
    if (np_iter == node_pool->end()) {
      auto con_iter = constraints.vertex_constraints.find(location);
      auto &sp = (*node_pool)[location];
      int l = 0;
      if (con_iter != constraints.vertex_constraints.end()) {
        for (auto &iter2 : con_iter->second) {
          int r = iter2.time - 1;
          if (r >= l) {
            SINode new_node;
            new_node.come_from.interval.r = -1;
            new_node.cost = Interval::oo;
            new_node.state = TISState(Interval(l, r), location.x, location.y);
            auto handle = open_set->push(new_node);
            new_node.handle = handle;
            sp.emplace_back(new_node);
          }
          l = iter2.time + 1;
        }
      }
      SINode new_node;
      new_node.come_from.interval.r = -1;
      new_node.cost = Interval::oo;
      new_node.state =
          TISState(Interval(l, Interval::oo), location.x, location.y);
      auto handle = open_set->push(new_node);
      new_node.handle = handle;
      sp.emplace_back(new_node);
      *node_list = &sp;
    } else {
      *node_list = &(np_iter->second);
    }
  }

  void SplitInterval(const Interval &interval, const Constraints &cons,
                     const Location &current_loc, const Location &next_loc,
                     std::vector<Interval> *split_intervals) {
    split_intervals->clear();
    if (interval.r < interval.l) return;
    auto con_iter =
        cons.edge_constraints.find(std::make_pair(next_loc, current_loc));
    int l = interval.l;
    if (con_iter != cons.edge_constraints.end()) {
      for (auto &iter2 : con_iter->second) {
        if (iter2.time < interval.l)
          continue;
        if (iter2.time > interval.r) break;
        int r = iter2.time - 1;
        if (r >= l) {
          split_intervals->emplace_back(Interval(l, r));
        }
        l = iter2.time + 1;
      }
    }
    if (interval.r >= l) {
      split_intervals->emplace_back(Interval(l, interval.r));
    }
  }

  void Update(const std::vector<Interval> &split_intervals, const int &cost,
              const TISState &from_state, std::list<SINode> *node_list,
              OpenSet *open_set) {
    if (split_intervals.size() == 0) return;
    int interval_id = 0;
    for (auto iter = node_list->begin(); iter != node_list->end(); iter++) {
      while (interval_id < static_cast<int>(split_intervals.size()) &&
             split_intervals[interval_id].r < iter->state.interval.l) {
        interval_id++;
      }
      if (interval_id == static_cast<int>(split_intervals.size())) return;
      if (iter->state.interval.r < split_intervals[interval_id].l) continue;
      if (iter->cost <= cost) continue;
      auto original_node = *iter;
      if (original_node.state.interval.l < split_intervals[interval_id].l) {
        auto new_node = original_node;
        new_node.state.interval = Interval(original_node.state.interval.l,
                                           split_intervals[interval_id].l - 1);
        auto handle = open_set->push(new_node);
        new_node.handle = handle;
        iter = node_list->insert(iter, new_node);
        iter++;
      }
      if (split_intervals[interval_id].r < original_node.state.interval.r) {
        auto new_node = original_node;
        new_node.state.interval = Interval(split_intervals[interval_id].r + 1,
                                           original_node.state.interval.r);
        auto handle = open_set->push(new_node);
        new_node.handle = handle;
        iter++;
        iter = node_list->insert(iter, new_node);
        iter--;
      }
      iter->state.interval.l = std::max(original_node.state.interval.l,
                                        split_intervals[interval_id].l);
      iter->state.interval.r = std::min(original_node.state.interval.r,
                                        split_intervals[interval_id].r);
      iter->cost = cost;
      iter->come_from = from_state;
      (*(original_node.handle)).state = iter->state;
      (*(original_node.handle)).cost = cost;
      open_set->update(original_node.handle);
    }
  }

  const Environment &env_;

  typedef std::unordered_map<TISState, HeapHandle_t, std::hash<TISState>>
      StateToHeap;
  TISState goal_state_;
};

typedef std::shared_ptr<SITree> SITreePtr;
};  // namespace mg_cbs

