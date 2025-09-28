# MKS_SERVO42D_control_Canbus
Sending messages over CANBUS to control revolutions and degrees of absolute motion.

The code MKS_SERVO42D_CAN_Spin_code.ino is a more basic route of the code that can be used to move the MKS_SERVO_42D motor with absolute precision. (The read encoder and rpm are currently not fully functioning).

The MKS_GIM code is designed to work on the same CAN bus lines as the gim, and so has the same frame structure as the ODrive code send frame. It can also be used to move the code to absolute positioning.

To use the Absolute positioning, use the command 'home', then activate a limit switch attached. Proceed to type commands rev and deg, followed by numbers. i.e. rev 5 would rotate 5 rotations from the set hom, which is done in the homing sequence with the limit switch previously. 
