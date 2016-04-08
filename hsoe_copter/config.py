# Configuration

import platform, glob

# The Arduino USB serial ID for linux
SERIAL_ARDUINO_ID = 'usb-Arduino__www.arduino.cc__Arduino_Mega_2560_7403335313035121D0D1-if00'

# Serial baud rate
SERIAL_ARDUINO_BAUD = 115200

# Location of the serial device
if ( platform.system( ) == "Linux" ):
    # On linux we can find the arduino by id
    SERIAL_PATH = os.path.join( "/dev/serial/by-id/", SERIAL_ARDUINO_ID )
elif ( platform.system( ) == "Darwin" ):
    # Check if we are directly connected to the apm
    paths = glob.glob( "/dev/tty.usbmodem*" )
    
    if ( len( paths ) > 0 ):
        SERIAL_PATH = paths[0]
    else:
        # If not, check if we are using telemetry
        paths = glob.glob( "/dev/tty.usbserial*" )

        if ( len( paths ) > 0 ):
            SERIAL_PATH = paths[0]
            SERIAL_ARDUINO_BAUD = 57600
        else:
            SERIAL_PATH = ""
elif ( platform.system( ) == "Windows" ):
    SERIAL_PATH = "???"
    
# Display heartbeat messages
DISPLAY_HEARTBEAT = False

# Display bad data
DISPLAY_BAD_DATA = True

# Display unknown data
DISPLAY_UNKNOWN_DATA = True
