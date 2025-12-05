## How does WaypointExecutive work?
    WaypointExecutive is a single-threaded loop that completes these steps in order.
        1. Interpret the Mission File
        2. Get a new mission task
        3. Get a new mission step
        4. Send a waypoint if needed onto the WaypointRosTopic.
        5. Check to see if the current step is completed.
        6. Check if any new changes of state for the robot have occurred.
        7. Service or adapt to the new change of state if needed.
        8. Loop steps 5 through 7 until the current step is completed.
        9. Loop steps 3 through 8 until the current task(all the steps of the task) is completed.
        10. Loop through 2 through 9 until all the tasks are completed.
        11. Success
    Its main inputs are a Mission Path file, position, vision, and battery interrupts.
    Its only output is a desired waypoint at the current time.
## What is a task and a step?
A task is a major action, expectation, or obstacle that the robot should perform, such as attempting to find and pass through the gate.
A step is part of a task that can include a waypoint that the robot needs to go to, a vision object or command that the robot needs to see and act on, or a manipulation objective that the robot needs to execute. 
## How is a step deemed completed?
    If the robot goes to the waypoint and either holds or touches it as instructed.
        OR
    If the robot has reached the maximum time of trying to complete the step.
        OR
    If the robot needs to hold its position(was never given a waypoint) for a time period.
        OR
    If it saw the required vision object and executed an action from it.
        OR
    If it executed the required manipulation task.
    
    If none of these conditions pass, it has not been deemed completed.

    If any one of these conditions passes, the controller moves on.
## What are the changes of states that the robot can experience?
    The battery is too low to continue.
    The Robot saw an object it needed to see.
    The Robot experienced a condition that needed to execute a manipulation step.
## How does the robot adapt or service these changes of state?
    If the battery is too low -> Surface, Make a Report, and Stop Operating.
    If the Robot saw an object -> Execute actions that have already been cleared to do so by pre-existing conditions, requirements, and commands.
    If the robot needs to trigger the manipulation code. -> Send the command through ROS.
# What are some examples of the controller's decision-making?
Imagine the robot needs to look for the gate, choose the right side, and needs to go through it. The Mission file should contain the waypoints of each step, with one of the steps being a vision command attached so that the robot can be interrupted once it spots the object and can attempt the next waypoint.
