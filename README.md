# SBMCS Firmware
Developed on Ubuntu 20.04 with ROS Noetic.
## Installation

### Using arduino CLI
- Installation [https://arduino.github.io/arduino-cli/installation/](https://arduino.github.io/arduino-cli/installation/)
- Add the adafruit board definition with
```
arduino-cli config init
```
- Edit the config file in the location shown and add Adafruit's board definition URL:```https://adafruit.github.io/arduino-board-index/package_adafruit_index.json``` 
- The file should contain a structure like 
```
board_manager:                                                                                                                                                
  additional_urls:                                                                                                                                            
  - https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
```
- Update the board index with ```arduino-cli core update-index```

### With Arduino IDE 
If not using arduino cli then just add the adafruit board url to the board manager that you access though the GUI

## Libraries and setup
The arduino cli makes library install easy for most libs. One exception is the rosserial package. To install this library you must have a working ROS workspace  with the rosserial metapackage built. Once this is done, run
```rosrun rosserial_arduino make_libraries.py PathToYourSketchbookLibraries```
which will generate the arduino library needed. For more information on getting a ROS catkin workspace setup, see KORA's install script [here](https://github.com/UKyKORA/LunaRover/blob/master/doc/ros_setup.sh) and the [ROS intallation homepage](http://wiki.ros.org/ROS/Installation)

Libraries to be downloaded by hand and placed in Arduino/libraries folder:
- https://github.com/BriscoeTech/Arduino-FreeRTOS-SAMD51 
- https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary

Libraries to install with CLI:
- Encoder: ```arduino-cli lib install Encoder```
  - this one requires a few changes to get it to build on the SAMD51 chip this SBMCS uses. As of writing this, there is a [PR in limbo](https://github.com/PaulStoffregen/Encoder/pull/34) that incorporates changes needed to support the interrupts that the SAMD51G19A chip has. I applied changes in the first two commits of this PR (21cf690 and 430004a) to my local copy of the Encoder library and its working great.
- PID: ```arduino-cli lib install PID```
- Madgwick: ```arduino-cli lib install Madgwick```
- NXPMotionSense ```arduino-cli lib install NXPMotionSense```

## Compilation

The SBMCS uses a SAMD51J19A with the arduino pin mapping of an Adafruit Feather M4 Express (yay opensource). This means when you want to compile SBMCS firmware that you need to target teh feather m4 express board.

In the arduino CLI, in the working directory of the sketch you want to compile (also accepts argument of sketch folder or file)
```
arduino-cli compile --fqbn adafruit:samd:adafruit_feather_m4
```

## Upload
With the proper serial port specified, of course:

```arduino-cli upload -p /dev/ttyACM0 --fqbn adafruit:samd:adafruit_feather_m4```
