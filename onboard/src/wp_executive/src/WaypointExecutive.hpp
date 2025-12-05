#include "../JSON_Parser/MissionAnalyzer.hpp"
#include "Task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstddef>
#include "../crs_common/position/position.hpp"
#include <fstream>
#include <iostream>
#include <memory>
#include <filesystem>
#include <optional>
#include <queue>
#include <utility>

/*
Notes:
  INT stands for interrupts.
  Theoretically, interrupts that occur while another interrupt is being serviced
or checked could be never attended to as the order in which the ServiceINTofTask
or services all the interrupts specifies priority. This could be solved by a
queue of Interrupts, but then there is a loss of actual priority. Thus a data
structure of the interrupts, preferably a max heap, that sorts itself is a
solution. Multithreading is another solution.


  Vision and Manipulation INT should be able to work together and apart. There
will probably be functions, but understanding the outputs and inputs from Vision
and Manipulation should solve this implementation problem.


*/
///@TODO:
// Tasks to Complete
// Interrupt Handling
//    Controller vs Task Interrupts Handling and Implementation Location -> Done
//    Then Finish up CheckINTofStep and ServiceINTofStep -> Done
//    Upload Design Image -> Not Needed.
// ROS2
//     Vision and Manipulation and Position ROS Topics -> Waiting for External
//     (Manipulation Done) Custom MSG for Float or just get_x ... -> Done
// Manipulation Code Checkup with ManipulationTask() updated. -> Needs Review
// StopWorking mechanism in Controller -> Done
// Timer Implementation Location Change to Task.hpp -> Done
// EndReport Details and Information Handling -> Done
//  Make the CurrentWaypointPtr a parameter for SendCurrentWaypoint.

struct Interrupts {
  bool SOCDANGER{false};
  bool BINS_SPOTTED{false};
  bool GATE_SPOTTED{true};
  bool DROP_INTO_BINS{false}; // READY_TO_DROP_INTO_BINS
  bool REEF_SHARK{false};
  bool TriggerManipSendCode{false};
  bool RanOutofTimeStep{false};
};


class WaypointExecutive : public rclcpp::Node {
public:
  WaypointExecutive(std::filesystem::path givenMissionPathPath)
      : Node("WaypointExecutiveNode") {
       
   MissionQueue = MissionAnalyzer(givenMissionPathPath);
    SetupROS();
    CurrentPositionPtr = std::make_shared<Position>();
  }
int Controller();
friend class WaypointExecutiveTest;
  void SetupROS();
  void StartorStopCameras();
  void SendCurrentWaypoint();
  void getNewMissionStep();
  void getNewMissionTask();
  bool isCurrentStepCompleted();
  void ManipulationStep(int code);
  void CheckINTofStep();
  void ServiceINTofStep();
  std::optional<std::filesystem::path> findFileInDirectory( const std::filesystem::path& start_path, 
    const std::string& directory_to_find, 
    const std::string& file_to_find);
  std::chrono::steady_clock::time_point timeInitalStep;
  // publisher of CurrentWaypointPtr topic.
  std::optional<bool> isSOCINT;
  std::vector<std::string> Last_Detected_Objects_Vector;
  std::mutex VisionVectorMutex;
  bool DidWeSeeObject(std::string key);
  std::shared_ptr<Position> CurrentPositionPtr;
  waypointPtr CurrentWaypointPtr;
  Task CurrentTask;
  Step CurrentStep;
  std::queue<Interrupts>
      Current_Interrupts; // Review the priority queue < opreator between two
                          // elements with void pointers.
  // Need to resolve the Time Elapsed and Counting.

  // callback ROS2 functions
  rclcpp::CallbackGroup::SharedPtr callbackINT;
  rclcpp::CallbackGroup::SharedPtr callbackPosition;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      WaypointPublisher;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      Camera1Publisher;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      Camera2Publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr CurrentTaskPub;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr Manipulation_Publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr VisionSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr SOCINTSub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr PositionSub;
  void VisionDetector(const std_msgs::msg::String::SharedPtr msg);
  void SOCIntCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void PositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  // Make This Pair or Class to save the interrupt details.
  bool StopWorking{false};
  MissionAnalyzer MissionQueue;
  bool MetPositionandTimeReq();
  void EndReport(Interrupts interrupt = Interrupts());
};
