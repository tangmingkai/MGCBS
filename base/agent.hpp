#pragma once
#include <iostream>
#include <vector>
#include <string>
#include "base/location.hpp"
#include "base/ts_state.hpp"
struct Agent {
  std::string agent_name;

  TSState start;
  std::vector<Location> goals;
  std::vector<Location> targets;
  Agent() = default;
  Agent(const std::string& agent_name, const TSState& start,
        const std::vector<Location>& goals)
      : agent_name(agent_name), start(start), goals(goals) {
    // targets = start + goals
    targets.emplace_back(start.GetLocation());
    targets.insert(targets.end(), goals.begin(), goals.end());
  }
  friend std::ostream& operator<<(std::ostream& os, const Agent& s) {
    os << s.agent_name << ": " << s.start << ", [";
    for (auto& goal : s.goals) {
      os << goal << ", ";
    }
    os << "]";
    return os;
    // return os << "(" << s.x << "," << s.y << ")";
  }
};
