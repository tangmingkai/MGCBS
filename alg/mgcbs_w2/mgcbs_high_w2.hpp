#pragma once

#include <map>
#include <memory>
#include <vector>
#include "env/environment.hpp"
#include "base/agent.hpp"
#include "alg/base_algorithm.hpp"
#include "alg/mgcbs_w2/mgcbs_mid_w2.hpp"
#include "alg/mgcbs_w2/distance_matrix_builder_w2.hpp"
#include "alg/mgcbs_w2/h_table_w2.hpp"
namespace mg_cbs_w2 {
class MGCBSHigh: public BaseAlgorithm {
 public:
  explicit MGCBSHigh(const Environment& environment)
      : env_(environment), middle_level_search_(environment),
        distance_matrix_builder_(environment) {}

  virtual bool Search(const std::vector<Agent>& agents,
                      std::vector<PlanResult>* solution) {
    solution->clear();
    HighLevelNode start;
    h_tables_.clear();
    h_tables_.reserve(agents.size());
    for (uint i = 0; i < agents.size(); i++) {
      auto &agent = agents[i];
      std::vector<std::vector<std::vector<int>>> full_distance_matrix;
      std::vector<std::vector<int>> target_distance_matrix;
      distance_matrix_builder_.BuildDistanceMatrix(agent, &full_distance_matrix,
                                                   &target_distance_matrix);
      h_tables_.emplace_back();
      h_tables_.back().Build(full_distance_matrix, target_distance_matrix);
    }
    start.solution.reserve(agents.size());
    start.constraints.reserve(agents.size());
    start.cost = 0;
    start.id = 0;
    for (size_t i = 0; i < agents.size(); ++i) {
      start.solution.emplace_back(std::make_shared<PlanResult>());
      start.constraints.emplace_back(std::make_shared<Constraints>());
      bool success = middle_level_search_.Search(
          agents[i], h_tables_[i], *start.constraints[i],
          &(*(start.solution[i])));
      if (!success) {
        return false;
      }
      start.cost += start.solution[i]->cost;
    }
    boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>> open;

    open.push(start);

    solution->clear();

    int id = 1;
    while (!open.empty()) {
      HighLevelNode p = open.top();
      open.pop();
      Conflict conflict;
      if (!env_.GetFirstConflict(p.solution, &conflict)) {
        for (auto& s : p.solution) solution->emplace_back(*s);
        return true;
      }
      std::map<size_t, Constraints> constraints;
      env_.CreateConstraintsFromConflict(conflict, constraints);
      for (const auto& c : constraints) {
        size_t i = c.first;
        HighLevelNode new_node = p;
        new_node.id = id;
        new_node.constraints[i] =
            std::make_shared<Constraints>(*(new_node.constraints[i]));
        new_node.constraints[i]->Add(c.second);
        new_node.cost -= new_node.solution[i]->cost;

        new_node.solution[i] = std::make_shared<PlanResult>();
        bool success = middle_level_search_.Search(
            agents[i], h_tables_[i],
            *(new_node.constraints[i]),  &(*(new_node.solution[i])));
        if (success &&
            !Check(*(new_node.solution[i]), *new_node.constraints[i])) {
          std::cout << "Error. Constraint not meet" << std::endl;
          exit(0);
        }
        new_node.cost += new_node.solution[i]->cost;
        if (success) {
          open.push(new_node);
        }
        ++id;
      }
    }
    return false;
  }
  bool Check(const PlanResult &result, const Constraints &con) {
    for (const auto& iter1 : con.vertex_constraints) {
      for (const auto& vc : iter1.second) {
        if (vc.time < static_cast<int>(result.states.size()) &&
            result.states[vc.time].GetLocation() == Location(vc.x, vc.y)) {
          std::cout << "Error. vc:" << vc << std::endl;
          for (auto& state : result.states) {
            std::cout << state << " ";
          }
          std::cout << std::endl;
          return false;
        }
      }
    }
    for (const auto& iter1 : con.edge_constraints) {
      for (const auto& ec : iter1.second) {
        if (ec.time + 1 < static_cast<int>(result.states.size()) &&
            result.states[ec.time].GetLocation() == Location(ec.x1, ec.y1) &&
            result.states[ec.time + 1].GetLocation() ==
                Location(ec.x2, ec.y2)) {
          std::cout << "Error. ec:" << ec << std::endl;
          for (auto& state : result.states) {
            std::cout << state << " ";
          }
          std::cout << std::endl;
          return false;
        }
      }
    }
    return true;
  }

 private:
  struct HighLevelNode {
    std::vector<PlanResultPtr> solution;
    std::vector<ConstraintsPtr> constraints;
    int cost;
    int id;

    bool operator<(const HighLevelNode& n) const {
      return cost > n.cost;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << std::endl;
      for (size_t i = 0; i < c.solution.size(); ++i) {
        os << "Agent: " << i << std::endl;
        os << " States:" << std::endl;
        for (size_t t = 0; t < c.solution[i]->states.size(); ++t) {
          os << "  " << c.solution[i]->states[t] << std::endl;
        }
        os << " Constraints:" << std::endl;
        os << c.constraints[i];
        os << " cost: " << c.solution[i]->cost << std::endl;
      }
      return os;
    }
  };


 private:
  const Environment &env_;
  MGCBSMid middle_level_search_;
  DistanceMatrixBuilder distance_matrix_builder_;
  std::vector<HTable> h_tables_;
};
};  // namespace mg_cbs_w2
