#pragma once

#include <vector>
#include <memory>
#include <string>
struct PlanResult {
  std::string agent_name;
  std::vector<TSState> states;
  std::vector<Action> actions;
  int cost;
  void Clear() {
    agent_name = "";
    states.clear();
    actions.clear();
    cost = 0;
  }
};

typedef std::shared_ptr<PlanResult> PlanResultPtr;
