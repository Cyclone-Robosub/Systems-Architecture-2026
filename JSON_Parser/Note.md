# How to write Mission files
## Important notes
**Javascript comments are not allowed PER the JSON standard.** Therefore, the documentation for the emission file presides in this file. In this document, "comments" refers to Javascript single line comments `//` and `/*n*/`

Thus, the "comments" in this document are purely for annotation purposes and **should not** be used inside a actual mission file.

## Structure
List of things to note:
- The type property limits the combination of properties that can be added to this field. See the types sections.
- The property might not be consistent across every section. If this is the cases, the docs will indicate that this is true. The "caveats" section shows what the behavior is depending on what type.
- What do the brackets and curly braces mean? See more on [basics on the json schema](https://json-schema.org/understanding-json-schema/basics). Note that the missionFile use the draft-7 addition schema.
```
{
    "tasks": [
        {
            "name": "a string",
            "steps": [
                {
                    "type": "one of the TYPES",

                    // Any property below this line in this scope are only for documentation purposes since some combination of them are mutually exclusive. See more in STEP

                    // SEE CAVEATS and STEP section
                    "hold_time": 1,
                    "max_time": 1,
                    "vision_command: "string", 
                    "manipulation_command": 1.
                    "time": 1.9
                }
            ]
        }
    ]
}
```

## Glossory
This section is a glossary of the the objects/properties inside of the Mission file.

- tasks: A array of task objects. There can only be one of these.
- name: The string representing the name of the task object
- steps: a array of step objects
- step: A object indicated by the curly braces inside fo the steps array. This. field does not have a key
- type: A enum. You may select one of the following values: "vision", "waypoint", "wait", or "manipulation.
    - **TYPE SHOULD BE THE FIRST FIELD OF THE STEP OBJECT" so that your ide can provide completion.
- hold_time: the amount of time in seconds to hold of the desired position once the destination is reached. Must be a positive non-zero value. 
    - **CAVEAT**:
    -  If the current step object is of type position or manipulation and the position array is given, then the hold_time field is mandatory. If the above is not true, it's optional
    - If position array is missing and the step type is vision, then hold_time is required
- max_time: the maximum amount of time that this step would run for.
    - **CAVEAT**:
    - If max_time is not provide, defaults to 39 seconds. This is set in the code and not the schema.
- position: 6-element position array: [x, y, z, roll, pitch, yaw]. x,y,z are in meters while roll, pitch, and yaw are euler angles. Support float-point. 
- vision_command: A string that will be processed as a "vision" command. Only available for steps with type "vision".
- manipulation_command: A integer that represents the sate of the manipulator (angle). Only available for steps with type "manipulation".


## Step
There are 4 types of step as described before. Here's how they look:
### waypoint
```
{
  "type": "waypoint",
  "position": [1.0, 2.0, 0.0, 0.0, 0.0, 90.0],
  "hold_time": 2.5, // mandatory if the position array, a optional, is given.
  "max_time": 1
}
```
### manipulation
```
{
          "type": "manipulation",
          "manipulation_command": 1,
          "position": [1.0, 2.0, 0.0, 0.0, 0.0, 90.0],
          "hold_time": 1, // mandatory if the position array, a optional, is given.
          "max_time": 2
}
```
### vision
{
          "type": "vision",
          "position": [1.0, 2.0, 0.0, 0.0, 0.0, 90.0],
          "vision_command": "BOX_DETECED",
          "hold_time": 1, // If position array is missing, then hold_time is reqiured
          "max_time": 1,
}
### wait
```
{
    "type": "wait",
    "time": 1.9
}