iirob_filters
=============

## ROS Distro Support
- ROS2 Foxy

## ROS2 Port
Only the gravity compensator and low pass filter have been ported. Both utilize dynamic parameters and require the parameters to be set before the function `configure()` is called.

## Tests
Run the current tests:
```
colcon test --packages-select iirob_filters --event-handlers console_direct+
```
