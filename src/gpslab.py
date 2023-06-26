#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#---GPS Puck Data Acquisition - creted by Kshama Dhaduti -

import rospy
import serial
from math import sin, pi
from std_msgs.msg import Float64
import time
import utm
from gps_driver.msg import custom

if __name__ == '__main__':
    SENSOR_NAME = "gps_sensor"
    pub= rospy.Publisher("custom_message",custom,queue_size=10)
    rospy.init_node('gps_sensor')
    
    # Get serial port and baudrate parameters from ROS parameter server
    serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate',4800)
    
    # Get sampling rate parameter from ROS parameter server
    sampling_rate = rospy.get_param('~sampling_rate',5.0)
     
    # Initialize the serial port for communication with the GPS sensor
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    
    # Log debug information
    rospy.logdebug("Using gps sensor on port "+serial_port+" at "+str(serial_baud))
    rospy.logdebug("Initializing sensor with *0100P4\\r\\n ...")
    
    # Calculate the number of samples to be taken based on the sampling rate
    sampling_count = int(round(1/(sampling_rate*0.007913)))
    
    # Sleep for a short duration to allow the sensor to initialize
    rospy.sleep(0.2)
    
    # Publishing longitude and latitude data
    rospy.loginfo("Publishing longitude and latitude.")
        
    # Create custom message object
    msg = custom()
    i = 1
    try:
        while not rospy.is_shutdown():
            # Set sequence number in message header
            msg.header.seq = i
            
            # Read a line of data from the serial port
            line = port.readline()
            line2 = line.decode('latin-1')
            
            if line == '':
                # If no data is received, log a warning
                rospy.logwarn("DEPTH: No data")
            else:
                if line2.startswith("$GPGGA"):
                    # If the line starts with "$GPGGA", it contains GPS data
                    
                    # Split the line into comma-separated values
                    s = line2.split(",")
                    
                    # Extract latitude, longitude, direction, UTC time, and altitude from the data
                    lat = s[2]
                    lon = s[4]
                    lat_dir = s[3]
                    lon_dir = s[5]
                    utc_time = s[1]
                    alt = s[9]
                    
                    # Convert latitude and longitude values to decimal degrees
                    degrees_lat = int(float(lat) / 100)
                    minutes_lat = float(lat) - (degrees_lat * 100)
                    degrees_lon = int(float(lon) / 100)
                    minutes_lon = float(lon) - (degrees_lon * 100)
                    dd_lat = float(degrees_lat) + float(minutes_lat) / 60
                    dd_lon = float(degrees_lon) + float(minutes_lon) / 60 
                    
                    # Adjust sign of latitude and longitude based on direction
                    if lon_dir == 'W':
                        dd_lon *= -1
                    if lat_dir == 'S':
                        dd_lat *= -1
                    
                    # Print latitude and longitude in decimal degrees
                    print("\n"+str(dd_lat)+" "+str(dd_lon))
 
                    # Convert latitude and longitude to UTM coordinates
                    utm_data3 = utm.from_latlon(dd_lat, dd_lon)
                    print(utm_data3)
                    
                    # Fill in the custom message with GPS data
                    msg.header.stamp = rospy.get_rostime()
                    msg.header.frame_id = "GPS_Data"
                    msg.latitude = dd_lat
                    msg.longitude = dd_lon
                    msg.altitude = float(alt)
                    msg.utm_easting = utm_data3[0]
                    msg.utm_northing = utm_data3[1]
                    msg.zone = float(utm_data3[2])
                    msg.letter_field = utm_data3[3]
                    
                    # Log the custom message
                    rospy.loginfo(msg)
                    
                    # Publish the custom message
                    pub.publish(msg)
                    
    except rospy.ROSInterruptException:
        # Close the serial port if ROS is interrupted
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")
