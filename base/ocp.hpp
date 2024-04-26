#pragma once
#include <boost/functional/hash.hpp>

struct VertexOcp {
  int time, x, y;
  VertexOcp() = default;
  VertexOcp(int time, int x, int y) : time(time), x(x), y(y) {}
  bool operator==(const VertexOcp& other) const {
    return time == other.time && x == other.x && y == other.y;
  }
};

namespace std {
template <>
struct hash<VertexOcp> {
  size_t operator()(const VertexOcp& c) const {
    size_t seed = 0;
    boost::hash_combine(seed, c.time);
    boost::hash_combine(seed, c.x);
    boost::hash_combine(seed, c.y);
    return seed;
  }
};
}  // namespace std

struct EdgeOcp {
  int time, x1, y1, x2, y2;
  EdgeOcp() = default;
  EdgeOcp(int time, int x1, int y1, int x2, int y2)
      : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
  bool operator==(const EdgeOcp& other) const {
    return time == other.time && x1 == other.x1 && y1 == other.y1 &&
           x2 == other.x2 && y2 == other.y2;
  }
};

namespace std {
template <>
struct hash<EdgeOcp> {
  size_t operator()(const EdgeOcp& c) const {
    size_t seed = 0;
    boost::hash_combine(seed, c.time);
    boost::hash_combine(seed, c.x1);
    boost::hash_combine(seed, c.y1);
    boost::hash_combine(seed, c.x2);
    boost::hash_combine(seed, c.y2);
    return seed;
  }
};
}  // namespace std
