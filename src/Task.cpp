#include "Task.hpp"
///@brief O(1) Algo and no conditional waiting. Save the current time in
/// timeInital.
void Step::StartTimer() {
  if (!isTimerOn) {
    timeInital = std::chrono::steady_clock::now();
    isTimerOn = true;
    std::cout << "running" << std::endl;
  }
}

///@brief O(1) Algo and no conditional waiting.
void Step::StopTimer() {
  if (isTimerOn) {
    auto deltaTime = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - timeInital)
                         .count();
    HoldWaypTime_TimeElapsed.emplace(
        std::make_pair(HoldWaypTime_TimeElapsed.value().first,
                       HoldWaypTime_TimeElapsed.value().second + deltaTime));
    isTimerOn = false;
  }
}
///@brief O(1) Algo and no conditional waiting. Add time to the Elapsed Time of
/// Current Step.
void Step::CalcTimer() {
  if (isTimerOn) {
    auto deltaTime = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - timeInital)
                         .count();
    HoldWaypTime_TimeElapsed.emplace(
        std::make_pair(HoldWaypTime_TimeElapsed.value().first,
                       HoldWaypTime_TimeElapsed.value().second + deltaTime));
    timeInital = std::chrono::steady_clock::now();
  }
}

void Step::ResetTimer(){
  if(isTimerOn){
    HoldWaypTime_TimeElapsed.value().second = 0.0;
    isTimerOn = false;
  }
}