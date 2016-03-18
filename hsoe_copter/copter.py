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

    @property
    def armed( self ):
        return self.mavSerial.motors_armed( )

    @property
    def hasAngle( self ):
        if ( self.pitch != 0 or self.roll != 0 or self.yaw != 0 ):
            return True
        else:
            return False
    
    @property
    def angle( self ):
        return ( self.pitch, self.roll, self.yaw );

    @property
    def angVel( self ):
        return ( self.pitchVel, self.rollVel, self.yawVel );

    @property
    def hasGPS( self ):
        if ( self.gpsLat != 0 or self.gpsLon != 0 ):
            return True
        else:
            return False
    
    @property
    def GPS( self ):
        return ( self.gpsLat, self.gpsLon, self.gpsAlt );

    @property
    def startTime( self ):
        return self.mavSerial.start_time

    @property
    def uptime( self ):
        return self.mavSerial.uptime

    @property
    def flightMode( self ):
        return self.mavSerial.flightmode

    @property
    def groundPressure( self ):
        return self.mavSerial.ground_pressure

    @property
    def groundTemp( self ):
        return self.mavSerial.ground_temperature

    @property
    def altitude( self ):
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
                self.gpsLat = msg.lat / 1.0e7
                self.gpsLon = msg.lon / 1.0e7
                self.gpsAlt = msg.alt / 1.0e3
                self.gpsHeading = msg.hdg
        
        return msg
