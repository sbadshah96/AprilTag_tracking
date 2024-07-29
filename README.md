# AprilTag_tracking

Repo for apriltag tracking on a drone.

Upload this package on your drone's ubuntu system with ROS2 Foxy.

## Instructions to run this package

- Create a workspace 'apriltag_ws/src' on drone's system.

- Clone all the packages under 'apriltag_ws/src'

- CLI commands:
    ```
    cd ~/apriltag_ws/
    ```
    ```
    colcon build
    ```
    ```
    source install/setup.bash
    ```
    ```
    ros2 run apriltag_tracking apriltag_tracking.py
    ```