#pragma once

#include <chrono>

class Stopwatch {
public:
  Stopwatch()
      : running(false), start_time(std::chrono::high_resolution_clock::now()),
        end_time(start_time) {}

  void Start() {
    running = true;
    start_time = std::chrono::high_resolution_clock::now();
  }

  void Stop() {
    running = false;
    end_time = std::chrono::high_resolution_clock::now();
  }

  double ElapsedMillis() const {
    using namespace std::chrono;
    duration<double> time_span;
    if (running) {
      time_span = duration_cast<duration<double>>(high_resolution_clock::now() -
                                                  start_time);
    } else {
      time_span = duration_cast<duration<double>>(end_time - start_time);
    }
    return time_span.count() * 1000;
  }

private:
  bool running;
  std::chrono::high_resolution_clock::time_point start_time;
  std::chrono::high_resolution_clock::time_point end_time;
};
