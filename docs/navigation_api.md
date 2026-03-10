# Arch-Nav available operations

## Arm
Arms the vehicle.

**Example**
```cpp
navigation_controller->arm()
```

## Disarm
Disarms the vehicle.

**Example**
```cpp
navigation_controller->disarm()
```

## Takeoff
Performs the takeoff operation to a given height.

| Parameters | Description                                                  |
|------------|--------------------------------------------------------------|
| height     | Relative (from the current position) takeoff height in meters.|

**Example**
```cpp
navigation_controller->takeoff(10.0)
```

## Waypoint following
Make the vehicle navigate through a set of given waypoints.

| Parameters        | Description                                                  |
|-------------------|--------------------------------------------------------------|
| waypoint list     | List of waypoints. Each waypoint is represented by a 3D <br>position in **GNSS** coordinates (See ROS2 GeoPose). |

**Example:**
```cpp
// Create a list of waypoints
std::vector waypoint_list = {
    GeoPose(40.7128, -74.0060, 10.0),  // New York
    GeoPose(34.0522, -118.2437, 15.0), // Los Angeles
    GeoPose(41.8781, -87.6298, 12.0)   // Barbate
};

// Execute waypoint following
navigation_controller->waypoint_following(waypoint_list);
```

## Land
Performs the land operation.

**Example**
```cpp
navigation_controller->land()
```

## Obtain Operation Status
Gives the operation status.

| Values     | Description                                              |
|------------|----------------------------------------------------------|
| RUNNING    | Operation still on course.                                |
| IDLE       | No active operations, vehicle available to perform a new operation.                                     |


**Example**
```cpp
navigation_controller->operation_status()
```

## Cancel Operation
Cancels the current active operation.

**Example:**
```cpp
navigation_controller->cancel_operation();
```

> **Note:** To perform a new operation, ensure that no other operation is currently running. You have two options:
> 
> **Option 1: Wait for the operation to complete**
> ```cpp
> // Start takeoff
> navigation_controller->takeoff(10.0);
> 
> // Wait until takeoff is complete
> while (navigation_controller->operation_status() == RUNNING) {
>     std::this_thread::sleep_for(std::chrono::milliseconds(100));
> }
> 
> // Now safe to start waypoint following
> navigation_controller->waypoint_following(waypoint_list);
> ```
> 
> **Option 2: Cancel the current operation**
> ```cpp
> // Start waypoint following
> navigation_controller->waypoint_following(waypoint_list);
> 
> // Cancel current operation to start a new one
> navigation_controller->cancel_operation();
> 
> // Wait until operation is fully cancelled
> while (navigation_controller->operation_status() == RUNNING) {
>     std::this_thread::sleep_for(std::chrono::milliseconds(100));
> }
> 
> // Now safe to start landing
> navigation_controller->land();
> ```
