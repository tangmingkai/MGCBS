#pragma once

#include <utility>
#include <boost/program_options.hpp>
struct Location {
  Location() = default;
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  bool operator!=(const Location& other) const {
    return std::tie(x, y) != std::tie(other.x, other.y);
  }
  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};

template <>
struct hash<std::pair<Location, Location> > {
 public:
  size_t operator()(const pair<Location, Location>& x) const {
    size_t seed = 0;
    boost::hash_combine(seed, hash<Location>()(x.first));
    boost::hash_combine(seed, hash<Location>()(x.second));
    return seed;
  }
};

}  // namespace std
