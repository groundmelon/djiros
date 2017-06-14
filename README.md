## A modified [TODO/OSDK(https://todo) ##

This is a modified version of TODO, which uses standard ros message types, providing limited function of TODO.

### Usage ###

* FindEigen.cmake from **ceres-solver** is used. Please modify it accordingly if you meet some problem about Eigen.

* Remember to pull the submodule OSDK, such as ```git submodule init && git submodule update```

* If you don't need mvBlueFOX synchronization, just [set ENABLE_DJIFOX to false](CMakeLists.txt#L22) to eliminate compile errors about mvBlueFOX drivers. If you need it, please install [the bluefox camera drivers](https://www.matrix-vision.com/USB2.0-single-board-camera-mvbluefox-mlc.html).

* Follow instructions of [DJI-OSDK](https://github.com/HKUST-Aerial-Robotics/OSDK) to install it into the system

* **launch/djiros.launch** and **launch/djifox.launch** will use environment variables to get APPID and ENCKEY. You can add your id and key in your launch file / set it to the environment / hardcode it in the source code.

* Configure A3 SDK as below
  ![A3 Configuration](docs/configuration.png)

* Please pay attention to ttyUSB name and privilege. An example for udev-rule file is provided in [docs/99-ftdi.rules](docs/99-ftdi.rules). A tutorial can be found at http://hintshop.ludvig.co.nz/show/persistent-names-usb-serial-devices/

### ROS Interfaces ###

#### Parameters ####
* serial_name             [string] : Path to the serial port device (e.g. /dev/ttyUSB0)
* baud_rate               [int]    : Baudrate for serial port
* app_id                  [int]    : App Id for dji sdk
* enc_key                 [string] : App Key for dji sdk
* sensor_mode             [bool]   : No activation and control is needed, just output imu, rc, gps, ...
* align_with_fmu          [bool]   : Use ticks from FMU/ ros::Time::now() when data is received.
* gravity                 [double] : scale multiplied on accelerometer output
* ctrl_cmd_stream_timeout [double] : timeout for judging if control command is streaming in or stopped
* ctrl_cmd_wait_timeout   [double] : timeout for waiting for control command after switch into api mode

#### Topics ###
* See the code and [official documents](https://developer.dji.com/onboard-sdk/documentation/) for published topics and their details.
* Subscriber "~ctrl" for control the drone
<!-- * Subscriber "~gimbal_ctrl" and "~gimbal_speed_ctrl" for control the gimbal -->

#### TODO ####

* Remove backwards.cpp

* Remove submodule when OSDK is released

* Update screenshot

* Add parameter explanations for bluefox