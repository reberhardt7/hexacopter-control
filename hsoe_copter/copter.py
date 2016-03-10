#! /usr/bin/env python

# pymavlink
# https://github.com/mavlink/mavlink

from pymavlink import mavutil

class Copter:
    def __init__( self, serial, baud ):
        print( "Opening mavlink connection..." )
        
        self.mavSerial = mavutil.mavlink_connection( serial, baud )
        self.mav = self.mavSerial.mav

        self.pitch = 0
        self.yaw = 0
        self.roll = 0

        self.pitchVel = 0
        self.yawVel = 0
        self.rollVel = 0
        
        self.gpsLat = 0
        self.gpsLon = 0
        self.gpsAlt = 0
        self.gpsHeading = 0
        
        self.waitHeartbeat( )
        self.requestStream( 4 )
        
        print( "Opened mavlink connection (" + self.mavSerial.device + ", "
               + str( self.mavSerial.baud ) + " baud)" )

    def __del__( self ):
        self.mavSerial.close( )
        print( "Closed mavlink connection" );
        
    def requestStream( self, rate ):
        self.mav.request_data_stream_send( self.mavSerial.target_system, self.mavSerial.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1 )
        
    def waitHeartbeat( self ):
        self.mavSerial.wait_heartbeat( )
        print( "Heartbeat received from system %u, component %u" % ( self.mavSerial.target_system, self.mavSerial.target_component ) )

    def disableArmChecks( self ):
        self.mavSerial.param_set_send( "ARMING_CHECK".encode( "UTF-8" ), 0 )

    def enableArmChecks( self ):
        self.mavSerial.param_set_send( "ARMING_CHECK".encode( "UTF-8" ), 1 )

    def arm( self ):
        self.mavSerial.arducopter_arm( )

    def disarm( self ):
        self.mavSerial.arducopter_disarm( )

    def isArmed( self ):
        return self.mavSerial.motors_armed( )

    def getAngle( self ):
        return ( self.pitch, self.roll, self.yaw );

    def getAngVel( self ):
        return ( self.pitchVel, self.rollVel, self.yawVel );
    
    def getGPS( self ):
        return ( self.gpsLat, self.gpsLon, self.gpsAlt );

    def getStartTime( self ):
        return self.mavSerial.start_time

    def getUptime( self ):
        return self.mavSerial.uptime
    
    def getFlightMode( self ):
        return self.mavSerial.flightmode

    def getGroundPressure( self ):
        return self.mavSerial.ground_pressure

    def getGroundTemp( self ):
        return self.mavSerial.ground_temperature

    def getAltitude( self ):
        return self.mavSerial.altitude
    
    def receiveMessage( self ):
        msg = self.mavSerial.recv_match( blocking = False )

        if ( msg is not None ):
            if ( msg.get_type( ) == "ATTITUDE" ):
                self.pitch = msg.pitch
                self.yaw = msg.yaw
                self.roll = msg.roll
                self.pitchVel = msg.pitchspeed
                self.yawVel = msg.yawspeed
                self.rollVell = msg.rollspeed
                
            if ( msg.get_type( ) == "GLOBAL_POSITION_INT" ):
                self.gpsLat = msg.lat
                self.gpsLon = msg.lon
                self.gpsAlt = msg.alt
                self.gpsHeading = msg.hdg
        
        return msg
