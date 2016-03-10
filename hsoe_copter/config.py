# Configuration

import platform

# The Arduino USB serial ID for linux
SERIAL_ARDUINO_ID = 'usb-Arduino__www.arduino.cc__Arduino_Mega_2560_7403335313035121D0D1-if00'

# Location of the serial device

if ( platform.system( ) == "Linux" ):
    # On linux we can find the arduino by id
    SERIAL_PATH = os.path.join( "/dev/serial/by-id/", SERIAL_ARDUINO_ID )
elif ( platform.system( ) == "Darwin" ):
    # Mac OSX, not sure if this is portable
    SERIAL_PATH = "/dev/tty.usbmodem1421"
elif ( platform.system( ) == "Windows" ):
    SERIAL_PATH = "???"

# Serial baud rate
SERIAL_ARDUINO_BAUD = 115200

# Display heartbeat messages
DISPLAY_HEARTBEAT = False

# Display bad data
DISPLAY_BAD_DATA = True

# Display unknown data
DISPLAY_UNKNOWN_DATA = False
