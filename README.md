## Vision Data API

This API provides information about detected objects in real-time.

### Endpoint: http://respi:5000/vision

- **Method:** `GET`

#### Response

The response is a JSON array containing information about detected objects:
Distance - In meters
Angles - In degrees (calculated from the center of the camera)

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
