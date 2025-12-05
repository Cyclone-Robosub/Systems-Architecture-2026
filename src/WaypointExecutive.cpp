#include "WaypointExecutive.hpp"

#include <chrono>
#include <fstream>
#include <memory>
#include <regex>

#include <nlohmann/json.hpp>

enum class ControllerStatus{
  COMPLETED_TASKS = 0,
  SOC_IS_LOW = 1
};
using json = nlohmann::json;

void WaypointExecutive::SetupROS() {
  callbackINT =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  callbackPosition =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto INTOPTIONS = rclcpp::SubscriptionOptions();
  INTOPTIONS.callback_group = callbackINT;
  auto PositionOptions = rclcpp::SubscriptionOptions();
  PositionOptions.callback_group = callbackPosition;

  auto VisionOptions = rclcpp::SubscriptionOptions();
  VisionOptions.callback_group = callbackINT;

  WaypointPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "waypoint_topic", 10);
       Camera1Publisher = this->create_publisher<std_msgs::msg::Bool>(
      "camera1_command_topic", 10);
       Camera2Publisher = this->create_publisher<std_msgs::msg::Bool>(
      "camera2_command_topic", 10);
  SOCINTSub = this->create_subscription<std_msgs::msg::Bool>(
      "SOCIntTopic", 10,
      std::bind(&WaypointExecutive::SOCIntCallback, this,
                std::placeholders::_1),
      INTOPTIONS);
  PositionSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "position_topic", 10,
      std::bind(&WaypointExecutive::PositionCallback, this,
                std::placeholders::_1),
      PositionOptions);
  CurrentTaskPub =
      this->create_publisher<std_msgs::msg::String>("CurrentTaskTopic", 10);
  VisionSub = this->create_subscription<std_msgs::msg::String>(
      "detections", 10,
      std::bind(&WaypointExecutive::VisionDetector, this,
                std::placeholders::_1),
      INTOPTIONS);
  Manipulation_Publisher =
      this->create_publisher<std_msgs::msg::Int64>("manipulationCommand", 10);
}

// Need to think about a better StopWorking mechanism.
int WaypointExecutive::Controller() {
  MissionQueue.parseJSONForMission();
  while (!MissionQueue.allTasksComplete()) {
    getNewMissionTask();
    while (!CurrentTask.steps_queue.empty()) {
      getNewMissionStep();
      SendCurrentWaypoint();
      while (!isCurrentStepCompleted()) {
        // if (CurrentTask.isInterruptable) { //Think about Hard vs Soft INT
        CheckINTofStep();  // Potentially Conditional Unresponsive Function
                           // (should not be as development continues)
        if (!Current_Interrupts.empty()) {
          ServiceINTofStep();
          if (StopWorking) {
            return 0;
          }
        }
        //}
      }
    }
  }
  EndReport();
  return 1;
}
///@brief O(1) Algo and no conditional waiting.
void WaypointExecutive::SendCurrentWaypoint() {
  auto message = std_msgs::msg::Float32MultiArray();
  message.data.resize(6);
  if (CurrentWaypointPtr != nullptr) {
    for (int i = 0; i < 6; ++i) {
      message.data[i] = (*CurrentWaypointPtr)[i];
    }
    WaypointPublisher->publish(message);
    std::cout << "sent" << std::endl;
  }
  // start the timer
}
///@brief O(1) and no conditional waiting. Returns False if the task should
/// still run.
bool WaypointExecutive::isCurrentStepCompleted() {
  if (MetPositionandTimeReq()) {
    return true;
  }

  // use the vision condition option to check if done?
  if (CurrentStep.VisionINTCommand_Serviced.has_value()) {
    if (CurrentStep.VisionINTCommand_Serviced.value().second) {
      return true;
    }
  }
  if (CurrentStep.ManipulationCodeandStatus.has_value()) {
    if (!CurrentStep.ManipulationCodeandStatus.value().second) {
      return true;
    }
  }
  // else
  return false;
}
///@brief Worst Case : O(How many objects were seen during the step). Pushes INT
/// instance to the
/// queue of Current Queue.
void WaypointExecutive::CheckINTofStep() {
  Interrupts generateINT;
  // Make this better if possible.
  bool didInterruptHappen = false;
  // check vision if needed -> Manipulation Tasks can be coded apart and along
  // side this vision requriment along with position and altitude.
  // check battery
  if (isSOCINT.has_value()) {
    isSOCINT.reset();
    generateINT.SOCDANGER = true;
    didInterruptHappen = true;
  }
  if (CurrentStep.VisionINTCommand_Serviced.has_value()) {
    const auto cmd = CurrentStep.VisionINTCommand_Serviced.value().first;
    if (cmd == "GATE") {
      if (DidWeSeeObject("Reef Shark")) {
        generateINT.REEF_SHARK = true;
        didInterruptHappen = true;
      }
    }
    if (cmd == "BINS_SPOTTED") {
      // Handle BINS_SPOTTED
      // e.g., reset flag, set interrupt
      // if(last_detected_class == "")
      generateINT.BINS_SPOTTED = true;
      didInterruptHappen = true;
    } else if (cmd == "DROP_INTO_BINS") {
      // Handle DROP_INTO_BINS
      // e.g., check altitude, reset flag, set manip code
     // generateINT.TriggerManipSendCode = true;
    //  didInterruptHappen = true;
    }
  }
  // listen to the vision topic;
  /*
  if (CurrentStep.ManipulationCodeandStatus.has_value()) {
    generateINT.TriggerManipSendCode = true;
    didInterruptHappen = true;
  }*/
  if (didInterruptHappen) {
    Current_Interrupts.push(generateINT);
  }
}

bool WaypointExecutive::DidWeSeeObject(std::string key) {
  std::lock_guard<std::mutex> lock(VisionVectorMutex);
  for (unsigned int i = 0; i < Last_Detected_Objects_Vector.size(); i++) {
    if (key == Last_Detected_Objects_Vector[i]) {
      return true;
    }
  }
  return false;
}

///@brief O(1) Algo and no conditional waiting. Service the INT. Clear the
/// Current Interrupt at the end.
void WaypointExecutive::ServiceINTofStep() {
  Interrupts ServiceINT = Current_Interrupts.front();
  std::cout << "service" << std::endl;
  if (ServiceINT.SOCDANGER) {
    // Battery WayPoint
  //  CurrentStep = Step();
    // CurrentStep.WaypointPointer = std::make_shared<waypointPtr>(); //
    // Creating a new waypoint.
   // SendCurrentWaypoint();
    EndReport(ServiceINT);
    StopWorking = true;
    
  }
  if (ServiceINT.RanOutofTimeStep) {
    EndReport(ServiceINT);
  }
  if (ServiceINT.REEF_SHARK) {
    CurrentStep.VisionINTCommand_Serviced.value().second = true;
  }
  if (ServiceINT.BINS_SPOTTED) {
    // Get Waypoint or coordinate from Vision
   	CurrentStep.VisionINTCommand_Serviced.value().second = true;
    SendCurrentWaypoint();
  }
  if (ServiceINT.TriggerManipSendCode) {
    ManipulationStep(CurrentStep.ManipulationCodeandStatus.value().first);
    CurrentStep.ManipulationCodeandStatus.value().second = true;
  }
  Current_Interrupts.pop();
}

void WaypointExecutive::getNewMissionTask() {
  CurrentTask = MissionQueue.popNextTask();
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = CurrentTask.name;
  CurrentTaskPub->publish(std::move(msg));
  std::cout << "new mission task " << CurrentTask.name << std::endl;
}

///@brief O(1) Algo and no conditional waiting. Has authority of changing
/// CurrentTask.
void WaypointExecutive::getNewMissionStep() {
  // fetch or predetermined waypoints.
  // Predetermined -> Waypoint Objects?
  if (!Last_Detected_Objects_Vector.empty()) {
    Last_Detected_Objects_Vector.clear();
  }
  CurrentStep = CurrentTask.steps_queue.front();
  CurrentTask.steps_queue.pop();
  if (CurrentStep.WaypointPointer != nullptr) {
    CurrentWaypointPtr = CurrentStep.WaypointPointer;
  }
  if(CurrentStep.VisionINTCommand_Serviced.has_value()){
    StartorStopCameras();
  }else{
    auto message = std_msgs::msg::Bool();
    message.data = false;
    Camera1Publisher->publish(message);
    Camera2Publisher->publish(message);
  }
  timeInitalStep = std::chrono::steady_clock::now();
  std::cout << "new mission step " << std::endl;
}

void WaypointExecutive::SOCIntCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  isSOCINT = msg->data;
}
void WaypointExecutive::VisionDetector(
    const std_msgs::msg::String::SharedPtr msg) {
  try {
    json parsed = json::parse(msg->data);

    for (const auto &item : parsed) {
      if (item.contains("class_name") && item["class_name"].is_string()) {
        std::lock_guard<std::mutex> lock(VisionVectorMutex);
        Last_Detected_Objects_Vector.push_back(
            item["class_name"].get<std::string>());
      }
    }

  } catch (const json::parse_error &e) {
    std::cerr << "could not parse vision data." << std::endl;
  }
}
void WaypointExecutive::PositionCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  // make this more efficient.
  CurrentPositionPtr =
      std::make_shared<Position>(msg->data[0], msg->data[1], msg->data[2],
                                 msg->data[3], msg->data[4], msg->data[5]);
}
void WaypointExecutive::ManipulationStep(int code) {
  // Send Manipulation Code over Publisher.
  auto manipulation_msg = std::make_unique<std_msgs::msg::Int64>();
  manipulation_msg->data = code;
  Manipulation_Publisher->publish(std::move(manipulation_msg));
}

///@brief O(1) Algo and no conditional waiting.
bool WaypointExecutive::MetPositionandTimeReq() {
  // check position var (include tolerance) with CurrentWaypointPtr
  // Check that we have to go to a position and we are currently running.
  if (CurrentWaypointPtr && CurrentPositionPtr) {
    if (CurrentWaypointPtr->AreWeThereYet(CurrentPositionPtr)) {
      // Check to see if we need to start the timer. (Don't check for time req
      // yet.)
      if (CurrentStep.HoldWaypTime_TimeElapsed.has_value()) {
        CurrentStep.StartTimer();
      }
    }
    // We ran off course or never reached it.
    else {
      CurrentStep.ResetTimer();
      //maxTime
      unsigned int deltaTime = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::steady_clock::now() - timeInitalStep)
                           .count();
      if (deltaTime >= CurrentStep.MaxTime) {
        Interrupts generateINT;
        generateINT.RanOutofTimeStep = true;
        Current_Interrupts.push(generateINT);
        return true;
      }
      return false;
    }
  }
  // We don't have something for all the data we need. Or we just need to wait
  // at our current position.
  else if (CurrentWaypointPtr == nullptr) {
    if (CurrentStep.HoldWaypTime_TimeElapsed.has_value()) {
      if (!CurrentStep.isTimerOn) {
        CurrentStep.StartTimer();
        std::cout << CurrentStep.HoldWaypTime_TimeElapsed.value().first
                  << std::endl;
      }
    } else {
      unsigned int deltaTime = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::steady_clock::now() - timeInitalStep)
                           .count();
      if (deltaTime >= CurrentStep.MaxTime) {
        Interrupts generateINT;
        generateINT.RanOutofTimeStep = true;
        Current_Interrupts.push(generateINT);
        return true;
      }
      return false;
    }
  } else {
    // The Robot has not started the current position topic.
    return false;
  }

  //}

  // Time Req after pos met or doesn't need to be met.
  if (CurrentStep.HoldWaypTime_TimeElapsed.has_value()) {
    CurrentStep.CalcTimer();
    if (CurrentStep.HoldWaypTime_TimeElapsed.value().first >
        CurrentStep.HoldWaypTime_TimeElapsed.value().second) {
      return false;
    } else {
      CurrentStep.StopTimer();
    }
  }

  std::cout << "met TIME and pos" << std::endl;
  return true;
}

void WaypointExecutive::StartorStopCameras(){
  auto messageCamera1 = std_msgs::msg::Bool();
  auto messageCamera2 = std_msgs::msg::Bool();
  messageCamera1.data = false;
  messageCamera2.data = false;
  if(CurrentStep.VisionINTCommand_Serviced.value().first == "BINS_SPOTTED"){
    messageCamera2.data = true;
  }
  Camera1Publisher->publish(messageCamera1);
  Camera2Publisher->publish(messageCamera2);
}


/// @brief: Will Exit itself after creating Report
void WaypointExecutive::EndReport(Interrupts interrupt) {
  std::ofstream ReportFile;
  ReportFile.open("../../End_Report.txt", std::ios::app);
  ReportFile << "___________START OF NOTIFICATION__________" << std::endl;
  ReportFile << "Reason for Report : ";
  if (MissionQueue.allTasksComplete()) {
    ReportFile << "All Tasks are Completed." << std::endl;
  }
  if (interrupt.SOCDANGER) {
    ReportFile << "State of Charge was low. Check Logs of SOC" << std::endl;
  }
  if (interrupt.RanOutofTimeStep) {
    ReportFile << "A step, " << CurrentWaypointPtr.get() <<", was skipped due to running out of time" << std::endl;
  }
  ReportFile << "___________END OF NOTIFICATION ___________" << std::endl;
  ReportFile << "\n";
  ReportFile.close();
}
// Function to find a specific file within a specific directory
std::optional<std::filesystem::path> WaypointExecutive::findFileInDirectory(
    const std::filesystem::path& start_path, 
    const std::string& directory_to_find, 
    const std::string& file_to_find) {
    
    // Create an empty path to store the found directory
    std::filesystem::path found_directory;

    // Use a recursive iterator to search all subdirectories
    for (const auto& entry : std::filesystem::recursive_directory_iterator(start_path)) {
        // Check if the current entry is a directory and has the correct name
        if (entry.is_directory() && entry.path().filename() == directory_to_find) {
            std::cout << "Found directory: " << entry.path() << std::endl;
            found_directory = entry.path();
            
            // Now, iterate within this found directory to find the file
            for (const auto& sub_entry : std::filesystem::directory_iterator(found_directory)) {
                if (sub_entry.is_regular_file() && sub_entry.path().filename() == file_to_find) {
                    std::cout << "Found file: " << sub_entry.path() << std::endl;
                    return sub_entry.path(); // Return the full path to the file
                }
            }
            // If the file wasn't found in the first "JSON_Parser" directory, 
            // you might want to continue searching for other "JSON_Parser" directories.
            // If you only expect one, you can break here.
        }
    }
    // If the loop finishes without finding the file, return an empty optional
    std::cout << "File or directory not found." << std::endl;
    return std::nullopt;
}
