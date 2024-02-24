# --- Message packet for Robot Controller status ---
# Last modified:    13 Jan 2024
# Author:           Fabian Kung
uint32  timestamp       # Time stamp, in RC controller ticks.
int32   rotater         # Rotation of right wheel, in no. of ticks.
int32   rotatel         # Rotation of left wheel, in no. of ticks. 
uint16  sen_ranger      # sensor: range right, in mm.
uint16  sen_rangel      # sensor: range left, in mm.
uint16  sen_rangef      # sensor: range front, in mm.
uint8   hwstatus1       # Hardware status flag 1.

# Note: 23/8/2023, here I decided to use uint8 instead of byte datatype 
# for single byte data. In ROS Humble if we declare the data as byte,
# we need to assign this in our code as (for example checksum)
# msg.checksum = b"1" 
# Whereas if we use uint8 keyword, we can assign with hexadecimal notation,
# which is clearer to the programmer. The other reason I think is speed. 
# However, I'm not certain as have not examine the codes for rclpy.