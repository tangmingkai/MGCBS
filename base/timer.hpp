#pragma once

#include <chrono>
#include <iostream>

class Timer {
 public:
  Timer()
      : start_(std::chrono::high_resolution_clock::now()),
        end_(std::chrono::high_resolution_clock::now()) {}

  void Reset() { start_ = std::chrono::high_resolution_clock::now(); }

  void Stop() { end_ = std::chrono::high_resolution_clock::now(); }

  double ElapsedSeconds() const {
    auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(
        end_ - start_);
    return timeSpan.count();
  }

 private:
  std::chrono::high_resolution_clock::time_point start_;
  std::chrono::high_resolution_clock::time_point end_;
};

class ScopedTimer : public Timer {
 public:
  ScopedTimer() {}

  ~ScopedTimer() {
    Stop();
    std::cout << "Elapsed: " << ElapsedSeconds() << " s" << std::endl;
  }
};
