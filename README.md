# SBMCS Firmware
Home for the code that runs on the KORA Small Bot Motor Control (SBMC) board. Hardware design files for the board can be found in the [SBMC_PCB](https://github.com/UKyKORA/SBMCS_PCB) repository. 


## Overview
Board functionality is broken down into tasks to be run in FreeRTOS.  We use the rosserial arduino package to communicate via UART to a single board computer running the ROS node ```rosserial_python serial_node.py```

Publised topics include:
- ```nav_msgs/Odometry``` on ```odom``` 
- ```sensor_msgs/Imu``` on ```imu_data```
- ```rover_msgs/SBMCTelemReeading``` on ```sbmc_telem```

Topics the firmware subscribes to:
- ```rover_msgs/SBMCServoSetting``` on ```/servo_setting```
- ```geometry_msgs/Twist``` on ```'drive_setting```

More information on ```rover_msgs``` can be found [here](https://github.com/UKyKORA/LunaRover/tree/master/src/rover_msgs).

To add: the different tasks, speed calibration procedure, PID tuning for motor control, mc33932 driver overview, shield class overview

## Setup



### Flashing the bootloader
In order to program the SBMC you must first upload the Adafruit UF2 bootloader. Since the pin mapping on the SBMC is matched to the [Adafruit Feather M4 Express](https://www.adafruit.com/product/3857), the bootloader is must be chosen similarly. The ```featherm4.bin```  in the ```bootloader``` directory is version 3.10.0 for the feather m4 found at [Adafruit's fork](https://github.com/adafruit/uf2-samdx1/releases) of microsofts UF2 bootloader.

### Install Arduino IDE/CLI
If you enjoy and/or are comfortable with terminal based work, the Arduino CLI makes using an external editor or console environment muc
#### CLI
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

#### IDE 
 - Download and install IDE for appropriate platform [https://www.arduino.cc/en/Main/Software](https://www.arduino.cc/en/Main/Software).
 - Add the adafruit board url to the board manager that you access though the GUI

### Libraries

Libraries to be downloaded by hand and placed in Arduino/libraries folder:
- https://github.com/BriscoeTech/Arduino-FreeRTOS-SAMD51 
- https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary

Libraries to install with IDE/CLI:
- Encoder: ```arduino-cli lib install Encoder```
  - this one requires a few changes to get it to build on the SAMD51 chip this SBMCS uses. As of writing this, there is a [PR in limbo](https://github.com/PaulStoffregen/Encoder/pull/34) that incorporates changes needed to support the interrupts that the SAMD51G19A chip has. I applied changes in the first two commits of this PR (21cf690 and 430004a) to my local copy of the Encoder library and its working great.
- PID: ```arduino-cli lib install PID```
- Madgwick: ```arduino-cli lib install Madgwick```
- NXPMotionSense ```arduino-cli lib install NXPMotionSense```

#### rosserial
 One exception is the rosserial package. To install this library you must have a working ROS workspace  with the rosserial metapackage built. Once this is done, run
```rosrun rosserial_arduino make_libraries.py PathToYourSketchbookLibraries```
which will generate the arduino library needed. For more information on getting a ROS catkin workspace setup, the [ROS melodic installation homepage](http://wiki.ros.org/ROS/Installation)



## Compilation

The SBMCS uses a SAMD51J19A with the arduino pin mapping of an Adafruit Feather M4 Express (yay opensource). This means when you want to compile SBMCS firmware that you need to target teh feather m4 express board.

In the arduino CLI, in the working directory of the sketch you want to compile (also accepts argument of sketch folder or file)
```
arduino-cli compile --fqbn adafruit:samd:adafruit_feather_m4
```

## Upload
With the proper serial port specified, of course:

```arduino-cli upload -p /dev/ttyACM0 --fqbn adafruit:samd:adafruit_feather_m4```
