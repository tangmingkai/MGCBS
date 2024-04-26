#pragma once

#include <iostream>

enum class Action {
  Unknown,
  Up,
  Down,
  Left,
  Right,
  Wait,
  WaitBeforeStart,
  Start
};


std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Up:
      os << "Up";
      break;
    case Action::Down:
      os << "Down";
      break;
    case Action::Left:
      os << "Left";
      break;
    case Action::Right:
      os << "Right";
      break;
    case Action::Wait:
      os << "Wait";
      break;
    case Action::WaitBeforeStart:
      os << "WaitBeforeStart";
      break;
    case Action::Start:
      os << "Start";
      break;
    case Action::Unknown:
      os << "Unknown";
  }
  return os;
}
