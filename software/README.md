# Prerequisites and installation
* install ros
* create a workspace
```
mkdir roboy-wheelchair
cd roboy-wheelchair
mkdir src
```
* clone
  https://github.com/jcmvbkbc/crosstool-NG.git
  https://github.com/Roboy/esp-wheelchair.git
* cd src
  https://github.com/ludwigthemad/rosserial_esp32.git

* build crosstool-NG as they describe it
* source /opt/ros/melodic/setup.bash
  go into roboy-wheelchair/. directory
```
catkin_make
source ./devel/setup.bash
rosrun rosserial_esp32 make_libraries.py $IDF_PATH/components/
```
*
```
export IDF_PATH=~/esp/ESP8266_RTOS_SDK
export PATH=$PATH:${IDF_PATH%/*}/xtensa-lx106-elf/bin
export PATH=$PATH:${IDF_PATH%/*}/ESP8266_RTOS_SDK/tools
export PATH=$PATH:${IDF_PATH%/*}/ESP8266_RTOS_SDK/components/esptool_py/esptool
```
* To config the ETH interface
```
idf.py menuconfig
```



First of all you need the [ESP8266 RTOS SDK](https://github.com/espressif/ESP8266_RTOS_SDK) installed on your system (More info about the installation procedure on their webpage).
Then make sure you have the tools from the package above and the xtensa-lx106-elf binaries on your PATH variable and that IDF_PATH points correctly to its installation directory.

I find it useful to add the following alias to set everything up quickly, especially when working with various idf versions:
```
alias get_idf8266='if [ -n "$IDF_PATH" ]
  then echo "IDF_PATH already set to $IDF_PATH"
    echo -n "Please check if this is the one you "
    echo "want to use!"
  else export IDF_PATH=${HOME}/git_sw/esp8266/ESP8266_RTOS_SDK
    export PATH=$PATH:${IDF_PATH%/*}/xtensa-lx106-elf/bin:\
${IDF_PATH%/*}/ESP8266_RTOS_SDK/tools:${IDF_PATH%/*}/ESP8266\
_RTOS_SDK/components/esptool_py/esptool
    echo "All ready!"
fi'
```
After this please create a catkin workspace to compile the esp libraries in, and clone the [rosserial_esp](https://github.com/ludwigthemad/rosserial_esp32) repo along the ros message sources you desire to include in the project in the src directory.

The resulting directory structure should look like this:
```
--catkin_workspace
|--src
   |--CMakeLists.txt
   |--some_messages1
   |--some_messages2
   |--rosserial_esp32
```

Then compile the workspace and follow the instructions in the rosserial_esp32 repo (or directly run make_libraries.py if you're feeling adventurous).

Check that the files have been generated correctly under $IDF_PATH/components/rosserial_esp32, and now the esp-wheelchair should compile with a simple 'make' command.

# Flashing the binaries to the board
## Connection to the system
First of all, open the wheelchair controller box (the one with the joystick) by unscrewing the 4 screws holding the top from underneath. Then search between the two electronics boards inside for a square 4pin connector wrapped in green tape. Now connect the USB adapter cable to it **making sure to match the ground pins to one another on both connectors!!** They are marked with black ink. Now you can connect the USB adapter to your PC.

### Important! Make sure before flashing code to the ESP that the wheelchair is NOT powered on, that means, the cable to the battery should be unplugged at all times when flashing.

## Flashing
After compilation is succesful, you can flash the code with 'make flash', but before that the esp8266 needs to boot in flash mode. For this you will need to press the BOOT button on the esp itself and at the same time press and release the RESET button. The buttons are located under the esp, the heatshrink has a cutout for them. (Pictures to follow)


# Known Issues
## crosstools-NG
won't build on Ubuntu 18.04 with gperf=3.1 due to a bug in the crosstool-ng repo
* nano crosstool-ng/kconfig/zconf.hash.c
then search for
kconf_id_lookup (register const char *str, register size_t len)
and change to
kconf_id_lookup (register const char *str, register unsigned int len)
