#include "MissionAnalyzer.hpp"
#include "JsonHelper.hpp"
#include <fstream>
#include <stdexcept>
using namespace nlohmann;
using namespace nlohmann::json_schema;
namespace fs = std::filesystem;

MissionAnalyzer::MissionAnalyzer() = default;

MissionAnalyzer::MissionAnalyzer(const MissionAnalyzer &other)
{
    filePath = other.filePath;
    copyQueue(other.mission);
}

MissionAnalyzer::MissionAnalyzer(std::filesystem::path filePath)
    : filePath(std::move(filePath)) {}

MissionAnalyzer &MissionAnalyzer::operator=(const MissionAnalyzer &other)
{
    if (this != &other)
    {
        filePath = other.filePath;
        copyQueue(other.mission);
    }
    return *this;
}

void MissionAnalyzer::copyQueue(const std::queue<Task> &other)
{
    std::queue<Task> tmp = other;
    mission = {};
    while (!tmp.empty())
    {
        mission.push(tmp.front());
        tmp.pop();
    }
}

Position MissionAnalyzer::makePositionFromJSON(nlohmann::json::reference jsonData)
{
    if (!jsonData.is_array() || jsonData.size() != 6)
    {
        throw std::runtime_error("Position must be an array of exactly 6 numbers.");
    }
    return Position(
        jsonData[0].get<double>(),
        jsonData[1].get<double>(),
        jsonData[2].get<double>(),
        jsonData[3].get<double>(),
        jsonData[4].get<double>(),
        jsonData[5].get<double>());
}

void MissionAnalyzer::parseJSONForMission()
{
    nlohmann::json missionJson = load_json_from_file(filePath);

    validate_with_detailed_errors(missionJson, filePath);
    // The tasks array
    nlohmann::json tasks = missionJson["tasks"];
    for(auto& taskJson : tasks) {
        Task task;
        task.name = taskJson["name"];
        for (auto& stepJson : taskJson["steps"]) {
            Step step;
            std::string taskType = stepJson["type"];

            if(taskType == "wait") {
                double holdTime = stepJson["time"].get<double>();
                step.HoldWaypTime_TimeElapsed = std::make_pair(holdTime, 0.0);
            }
            if (stepJson.contains("position")) {
                Position pos = makePositionFromJSON(stepJson["position"]);
                step.WaypointPointer = std::make_shared<Position>(pos);
            }
            // Hold_time
            if (stepJson.contains("hold_time")) {
                double holdTime = stepJson["hold_time"].get<double>();
                step.HoldWaypTime_TimeElapsed = std::make_pair(holdTime, 0.0);
            }

            // MaxTime
            if (stepJson.contains("max_time")) {
                step.MaxTime = stepJson["max_time"].get<unsigned int>();
            }

            // Vision command
            if (taskType == "vision" && stepJson.contains("vision_command")) {
                std::string cmd = stepJson["vision_command"].get<std::string>();
                step.VisionINTCommand_Serviced = std::make_pair(cmd, false);
            }

            // Manipulation command
            if (taskType == "manipulation" && stepJson.contains("manipulation_command")) {
                int cmd = stepJson["manipulation_command"].get<int>();
                step.ManipulationCodeandStatus = std::make_pair(cmd, false);
            }

            task.steps_queue.push(step);
        }
        mission.push(task);
    }
}

Task MissionAnalyzer::popNextTask()
{
    if (mission.empty())
    {
        throw std::runtime_error("No more tasks in mission queue.");
    }
    Task next = mission.front();
    mission.pop();
    return next;
}

bool MissionAnalyzer::allTasksComplete()
{
    return mission.empty();
}
