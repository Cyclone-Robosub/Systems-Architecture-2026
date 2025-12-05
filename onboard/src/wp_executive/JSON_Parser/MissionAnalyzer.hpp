#ifndef MISSION_ANALYZER_HPP
#define MISSION_ANALYZER_HPP

#include "../crs_common/position/position.hpp"
#include "Task.hpp"
#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>
#include <queue>
#include <string>
#include <filesystem>

// Forward declaration
struct Task;

class MissionAnalyzer {
public:
  MissionAnalyzer();
  MissionAnalyzer(const MissionAnalyzer& other); 
  MissionAnalyzer(std::filesystem::path filePath);
  void parseJSONForMission();
  Task popNextTask();
  bool allTasksComplete();
  MissionAnalyzer& operator=(const MissionAnalyzer& other);
private:
  std::filesystem::path filePath;
  std::queue<Task> mission;
  Position makePositionFromJSON(nlohmann::json::reference jsonData);
  void copyQueue(const std::queue<Task>& other);
};
#endif