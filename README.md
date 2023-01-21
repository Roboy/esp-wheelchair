# ESP8266 + RTOS + ROS1
## Prerequisites
- install ROS1 melodic

## Set up the environment
Setup ESP8266 RTOS SDK (see https://docs.espressif.com/projects/esp8266-rtos-sdk/ for more details)
```bash
# get dependencies
sudo apt-get install gcc git wget make libncurses-dev flex bison gperf python python-serial

mkdir -p ~/esp
cd ~/esp

# get the toolchain
wget https://dl.espressif.com/dl/xtensa-lx106-elf-linux64-1.22.0-92-g8facf4c-5.2.0.tar.gz
tar -xzf xtensa-lx106-elf-linux64-1.22.0-92-g8facf4c-5.2.0.tar.gz

# get the SDK
git clone --recursive https://github.com/espressif/ESP8266_RTOS_SDK.git

export IDF_PATH=~/esp/ESP8266_RTOS_SDK
export PATH=$PATH:${IDF_PATH%/*}/xtensa-lx106-elf/bin
export PATH=$PATH:${IDF_PATH%/*}/ESP8266_RTOS_SDK/tools
export PATH=$PATH:${IDF_PATH%/*}/ESP8266_RTOS_SDK/components/esptool_py/esptool

python -m pip install --user -r $IDF_PATH/requirements.txt

```

## Add ROS support
```bash
mkdir catkin_ws/src -p
cd catkin_ws/src
git clone https://github.com/ludwigthemad/rosserial_esp32.git
cd catkin_ws 
catkin_make
source catkin_ws/devel/setup.bash
rosrun rosserial_esp32 make_libraries.py $IDF_PATH/components/
```

## Compile the app
```bash

git clone https://github.com/Roboy/esp-wheelchair.git -b wifi-with-cytron
cd esp-wheelchair/software

# configure the serial port ESP is connected to
make menuconfig # navigate to Serial flasher config > Default serial port and enter e.g. ttyUSB0

make # to build the project or
make flash # to flash
make monitor # to open serial monitor

```

__When compiling the app, make sure you have the correct versions installed__. The following should be printed as the first output of `make` command:

```
Toolchain version: crosstool-ng-1.22.0-100-ge567ec7
Compiler version: 5.2.0
```
