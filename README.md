## Vision Data API

This API provides information about detected objects in real-time.

### Endpoint: http://respi:5000/vision

- **Method:** `GET`

#### Response

The response is a JSON array containing information about detected objects:

* Distance - In meters

* Angles - In degrees (calculated from the center of the camera)

Example response:

```json
[
    {
        "class_name": "note",
        "distance": 2.5,
        "angle": -12.3
    },
    {
        "class_name": "note",
        "distance": 5.7,
        "angle": 8.1
    }
]
```
## VisionDrive Subsystem

The `VisionDrive` command is responsible for integrating vision data into the control of the robot's Swerve drive system. It fetches distance and angle information from the `Vision` subsystem and calculates the appropriate forward and turn speeds to drive towards the detected object.

## Shooter Command
The `ShooterCommand` controlling the shooter subsystem. It provides a convenient way to orchestrate the shooter's pivot and flywheel angles, initiate shooting, and handle the stopping of the shooter subsystem.

## Please add important info (if neccecry) about your code here, i just added the shooter as an example (not my code).
