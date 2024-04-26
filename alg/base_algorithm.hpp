#pragma once

#include "bits/functional_hash.h"
#include "../base/plan_result.hpp"

class BaseAlgorithm {
 public:
  virtual bool Search(const std::vector<Agent>& agents, std::vector<PlanResult>* solution) = 0;
};