#ifndef TASK_HPP
#define TASK_HPP

#include "../crs_common/position/position.hpp"
#include "../JSON_Parser/MissionAnalyzer.hpp"
#include <chrono>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <utility>

typedef std::shared_ptr<Position> waypointPtr;

struct MissionAnalyzer;
struct Step {
  waypointPtr WaypointPointer;
  std::optional<std::pair<std::string, bool>> VisionINTCommand_Serviced;
  bool isInterruptable{true}; // Think About Hard vs Soft INT
  bool doBarrelRoll{false};    // Will also need to be more robust
  bool stopWorking{false};
  std::optional<std::pair<int, bool>> ManipulationCodeandStatus;

  // Make sure that the second of the pair when initalized is set to 0.
  std::optional<std::pair<double, double>> HoldWaypTime_TimeElapsed;
  unsigned int MaxTime = 39;
  // const static Waypoints pre-determined of vector?

  void StartTimer();
  void StopTimer();
  void CalcTimer();
  void ResetTimer();
  bool isTimerOn{false};
  std::chrono::steady_clock::time_point timeInital;
};

struct Task {
  std::string name;
  std::queue<Step> steps_queue;
};

#endif