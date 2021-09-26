# Python-VectorNav

This library reads VectorNav IMU binary messages through a MacOS device in Python, dealing with checksum.
This library does not use any OS dependant libraries, so it could be used for any OS that runs Python.

Current official VectorNav drivers do not support reading sensor messages in Python from Mac OS.
It is not that hard to write your own code if your are simply reading data as ASCII instead of binary messages.
But using ASCII is not ideal if you want read data at a higher frequency.

The example provided in this code parses data sent from the VN100 in binary format.
However, this code is not limited to VN100, and can be used for other VectorNav sensors.
The downside of this not being the official drivers is that you have to do some configurations manually.

If you are looking for reading VectorNav messages from an Arduino, please checkout [https://github.com/fdcl-gwu/arduino-vn100](https://github.com/fdcl-gwu/arduino-vn100)

## IMU Configuration
Below are the configurations required to be set on the VN100 to run the provided code without any alterations.

You can use any serial communication software (e.g.: Serial Monitor on Arduino IDE) to set following configurations on the IMU.
If you are using the Arduino Serial Monitor, make sure to change the line ending setting in the bottom of the Serial Monitor to `Newline`.
If you need any other parameters, you must change the group field values as described in the sensor manual, as well as the message parsing on VectorNav class.

```
$VNASY,0*XX                 // stop async message printing
$VNWRG,06,0*XX              // stop ASCII message outputs
$VNWRG,75,2,8,01,0128*XX    // output binary message (see below for details)
$VNCMD*XX                   // enter command mode
system save                 // save settings to flash memory
exit                        // exit command mode
$VNASY,1*XX                 // resume async message printing
```

### Configuring IMU Binary Message
Command | Register | Serial Port | Frequency | Group | Output Message | Checksum
------- | -------- | ----------- | --------- | ----- | -------------- | ---------
$VNWRG  | 75       | 2           | 8         | 01    | 0128           | *XX
Write to register command | Register number for the output binary message | This depends on the cable you use | divide 800 by this value get the required frequency, this example is for 100 Hz | Output group as defined in the manual | data categories from the group, this example is for b100101000 = 0x0128 | Use XX for unknown checksum values 

## Running the Code
1. Configure the IMU as described above.
1. Update the serial port and baud rates in test_vn100.py
1. Run the code: `python test_vn100.py`