# --- Message packet for Robot Controller status ---
# Last modified:    9 Dec 2023
# Author:           Fabian Kung
uint8   command     # Command to Robot Controller.
uint8   arg1        # Argument 1.
uint8   arg2        # Argument 2.
uint8   arg3        # Argument 3.
uint8   arg4        # Argument 4.  
uint8   checksum    # Checksum.