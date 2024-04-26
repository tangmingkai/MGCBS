#pragma once
#include <iostream>
#include <algorithm>

struct Interval {
  Interval() = default;
  Interval(int l, int r) : l(l), r(r) {}
  // [l,r]
  int l, r;

  bool empty() const { return l > r; }

  bool operator==(const Interval& s) const { return l == s.l && r == s.r; }

  Interval operator-(const int& d) const {
    Interval v;
    v.l = std::max(0, l - d);
    if (r == oo)
      v.r = oo;
    else
      v.r = r - d;

    return v;
  }

  Interval operator+(const int& d) const {
    Interval v;
    v.l = l + d;
    if (r == oo)
      v.r = oo;
    else
      v.r = r + d;
    return v;
  }
  bool IsIn(const int& t) const { return l <= t && t <= r; }
  friend std::ostream& operator<<(std::ostream& os, const Interval& s) {
    return os << "[" << s.l << ", " << s.r << "]";
    // return os << "(" << s.x << "," << s.y << ")";
  }

  static const int oo;
};
const int Interval::oo = 200000000;

namespace std {
template <>
struct hash<Interval> {
  size_t operator()(const Interval& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.l);
    boost::hash_combine(seed, s.r);
    return seed;
  }
};
}  // namespace std
