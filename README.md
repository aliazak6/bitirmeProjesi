# Ball Plate Balance System
ELE 495 2021-2022 Summer 1. Group Project
## Establishing bluetooth connection
hc-05 paired via bluetoothctl 

$ sudo rfcomm bind rfcomm0 XX:XX:XX:XX:XX:XX 

$ chmod a+rw /dev/rfcomm0

## servo_test
servo_test.py script can be used for testing servo angles and determining 
initial angles of realized physical system.

## color_test

color_test.py script can be used for determining image process color values.
Using the interactive GUI, one can change HSV color space values of the code and 
use the platform with any color of ball it wants.

## main

main.py file is the main file of this project, when executed system starts 
and servo motors try to balance the ball. Meanwhile bluetooth data sent to 
/dev/rfcomm0 

## bluetooth

bluetooth.ino file must be loaded to arduino card. Pin layout described inside
the module.
