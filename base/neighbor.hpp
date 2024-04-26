#pragma once

#include "base/tis_state.hpp"
#include "base/action.hpp"
struct Neighbor {
  Neighbor(const Location& location, const Action& action, int cost)
      : location(location), action(action), cost(cost) {}
  //! neighboring state
  Location location;
  //! action to get to the neighboring state
  Action action;
  //! cost to get to the neighboring state
  int cost;
};

struct NeighborDir {
  NeighborDir() = default;
  NeighborDir(int dx, int dy, Action action, int cost)
      : dx(dx), dy(dy), action(action), cost(cost) {}
  int dx;
  int dy;
  Action action;
  int cost;
};
