#! /usr/bin/env python

import os, sys
import config, copter

print( '---===HSOE Copter===---' )

copter = copter.Copter( config.SERIAL_PATH, config.SERIAL_ARDUINO_BAUD )
copter.disableArmChecks( )
copter.arm( )

running = True
lastBad = False

while running == True:
    try:
        msg = copter.receiveMessage( )
        
        if ( msg is not None ):
            
            if( msg.get_type( ) == "BAD_DATA" ):
                if ( config.DISPLAY_BAD_DATA ):
                    sys.stdout.write( msg.data )
                    sys.stdout.flush( )
                    lastBad = True
            else:
                if ( lastBad ):
                    sys.stdout.write( '\n' )
                    sys.stdout.flush( )

                lastBad = False
                
                if ( msg.get_type( ) == "HEARTBEAT" ):
                    if ( config.DISPLAY_HEARTBEAT ):
                        print( "Heartbeat" )
                    
                elif ( msg.get_type( ) == "STATUSTEXT" ):
                    print( "Status: " + msg.text.decode( "utf-8" ) )
                
                elif ( config.DISPLAY_UNKNOWN_DATA ):
                    print( msg )
                
    except KeyboardInterrupt:
        running = False
