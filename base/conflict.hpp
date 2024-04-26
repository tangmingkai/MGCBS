#pragma once

#include <iostream>
struct Conflict {
  Conflict() {
    type = Vertex;
    time = agent1 = agent2 = x1 = y1 = x2 = y2 = 0;
  }
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;

  int x1;
  int y1;
  int x2;
  int y2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << "Vertex(" << c.agent1 << ", " << c.agent2 << ", " << c.time
                  << ", " << c.x1 << ", " << c.y1 << ")";
      case Edge:
        return os << "Edge(" << c.agent1 << ", " << c.agent2 << ", " << c.time
                  << ", " << c.x1 << ", " << c.y1 << ", " << c.x2 << ", "
                  << c.y2 << ")";
    }
    return os;
  }

  bool operator==(const Conflict& other) const {
    return x1 == other.x1 &&
           y1 == other.y1 &&
           x2 == other.x2 &&
           y2 == other.y2 &&
           agent1 == other.agent1 &&
           agent2 == other.agent2 &&
           type == other.type && time == other.time;
  }
};

namespace std {
template <>
struct hash<Conflict> {
  size_t operator()(const Conflict& c) const {
    size_t seed = 0;
    boost::hash_combine(seed, c.time);
    boost::hash_combine(seed, c.agent1);
    boost::hash_combine(seed, c.agent2);
    boost::hash_combine(seed, c.type);
    boost::hash_combine(seed, c.x1);
    boost::hash_combine(seed, c.y1);
    boost::hash_combine(seed, c.x2);
    boost::hash_combine(seed, c.y2);
    return seed;
  }
};
}  // namespace std
