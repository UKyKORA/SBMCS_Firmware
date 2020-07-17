# SBMCS Firmware

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

### If not using arduino cli then just add the adafruit board url to the board manager that you access though the GUI

## Compilation

The SBMCS uses a SAMD51J19A with the arduino pin mapping of an Adafruit Feather M4 Express (yay opensource). This means when you want to compile SBMCS firmware that you need to target teh feather m4 express board.

In the arduino CLI, in the working directory of the sketch you want to compile (also accepts argument of sketch folder or file)
```
arduino-cli compile --fqbn adafruit:samd:adafruit_feather_m4
```

## Upload
With the proper serial port specified, of course:

```arduino-cli upload -p /dev/ttyACM0 --fqbn adafruit:samd:adafruit_feather_m4```
