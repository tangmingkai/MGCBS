#pragma once

#include <iostream>
#include "base/interval.hpp"
#include "base/location.hpp"
struct TISState {
  TISState() = default;
  TISState(const Interval& interval, int x, int y)
      : interval(interval), x(x), y(y) {}

  bool operator==(const TISState& s) const {
    return interval == s.interval && x == s.x && y == s.y;
  }
  // // when time < 0, it means the agent is not in the scene
  // bool EqualExceptTime(const TISState& s) const {
  //   return x == s.x && y == s.y;
  // }

  friend std::ostream& operator<<(std::ostream& os, const TISState& s) {
    return os << "(" << s.interval <<", "<< s.x << ", " << s.y << " " << ")";
    // return os << "(" << s.x << "," << s.y << ")";
  }
  Location GetLocation() const { return Location(x, y); }
  Interval interval;
  int x;
  int y;
};

namespace std {
template <>
struct hash<TISState> {
  size_t operator()(const TISState& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, hash<Interval>()(s.interval));
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std
