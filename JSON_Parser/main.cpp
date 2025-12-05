// #include <iostream>
// #include <string>
// #include "MissionAnalyzer.hpp"

// int main() {
//     auto analyzer = MissionAnalyzer("MissionPath.JSON");
//     analyzer.parseJSONForMission();

    
//     for (int taskCount = 0; !analyzer.allTasksComplete(); taskCount++) {
//         Task task = analyzer.popNextTask();
        
//         std::cout << "\nTask " << taskCount << ": " << task.name << std::endl;
        
//         int stepCount = 0;
//         std::queue<Step> steps_queue = task.steps_queue;
        
//         while (!steps_queue.empty()) {
//             Step step = steps_queue.front();
//             steps_queue.pop();
//             stepCount++;
            
//             std::cout << "  Step " << stepCount << ":" << std::endl;
            
//             if (step.WaypointPointer) {
//                 std::cout << "      Is waypoint: Yes" << std::endl;
//             } else {
//                 std::cout << "      Is waypoint: No" << std::endl;
//             }
            
//             std::cout << "      Needs vision: " << (step.VisionINTCommand_Serviced.value.first) << std::endl;
//             std::cout << "      Is interruptable: " << (step.isInterruptable ? "Yes" : "No") << std::endl;
//             std::cout << "      Do barrel roll: " << (step.doBarrelRoll ? "Yes" : "No") << std::endl;
            
//             if (step.HoldWaypTime_TimeElapsed.has_value()) {
//                 std::cout << "      Hold time: " << step.HoldWaypTime_TimeElapsed->first << " seconds" << std::endl;
//             }

//             if (step.stopWorking) {
//                 std::cout << "      Control signal: STOP" << std::endl;
//             }
//         }
//     }
    
//     std::cout << "Mission Parser Demo Complete Successfully" << std::endl;
    
//     return 0;
// }
