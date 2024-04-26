#pragma once

#include <iostream>
#include "base/location.hpp"

struct TSState {
  TSState() = default;
  TSState(int time, int x, int y) : time(time), x(x), y(y) {}

  bool operator==(const TSState& s) const {
    return time == s.time && x == s.x && y == s.y;
  }
  bool operator!=(const TSState& s) const {
    return time != s.time || x != s.x || y != s.y;
  }
  // when time < 0, it means the agent is not in the scene
  bool EqualExceptTime(const TSState& s) const {
    return  x == s.x && y == s.y;
  }

  friend std::ostream& operator<<(std::ostream& os, const TSState& s) {
    return os << "([" << s.time <<"], "<< s.x << ", " << s.y << ")";
  }
  Location GetLocation() const { return Location(x, y); }
  int time;
  int x;
  int y;
};

namespace std {
template <>
struct hash<TSState> {
  size_t operator()(const TSState& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std
