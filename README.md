# Dualsense ROS
This package contains ROS2 driver for dualsense controller. The package is based on [pydualsense](https://github.com/flok/pydualsense/tree/master) library. Head to this link for package documentation

## Package status
Currently the package creates a simple publisher node that publishes basic button data from the controller. To run it:
 - clone this package to ros2 workspace
 - Connect the controller to your PC via bluetooth
 - build `dualsense_ros` package
 - run `ros2 run dualsense_ros dualsense_ros_node`
 - in separate terimnal run `ros2 topic echo /joy`

### Button mapping
Button mappings are desribed in configuration .yaml files in `config` directory.

## Examples
For further development of this package I strongly reccomend using [those examples](https://github.com/flok/pydualsense/tree/master/examples) as a reference point

## TODO
 - [LED](https://github.com/flok/pydualsense/blob/master/examples/leds.py), [vibration and adaptive trigger](https://github.com/flok/pydualsense/blob/master/examples/effects.py) control using ROS2 services
 - Reading touchpad input and sending it via topic ([this example, lines 31,32](https://github.com/flok/pydualsense/blob/master/examples/read_all_input_channels.py))
 - Research if audio support is possible (I did not find it in the used library)
 - Handle gyroscope and accelerometer data and send it via topic ([this example, lines 33,34](https://github.com/flok/pydualsense/blob/master/examples/read_all_input_channels.py))
 - Handle battery level

 This list is a guideline and NOT a list of requirements. Feel free to modify it as you like. 