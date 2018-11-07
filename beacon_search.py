#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Copyright {2018} {Bluebird Mountain | Moritz Obermeier}

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
import dronekit
import argparse
import traceback
import threading
import time
import sys
import math
import logging
import geopy
import geopy.distance as distance
 
from pymavlink import mavutil
from yaml import load

#parameters now come from PARAMETERS.yaml

def load_parameters():
  try:
    stream = file('/home/pi/src/moobsen/beacon_search/PARAMETERS.yaml', 'r')
  except:
    import os
    stream = file(os.getcwd()+'/PARAMETERS.yaml','r')
  params = load(stream)
  logging.info('Loaded parameters: %s' % params)
  return params

BEACON_INPUT_PIN = 17 #global GPIO PIN number (I know)

def setup_buttons():
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(BEACON_INPUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def goto_position_target_global_int(aLocation, vehicle):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified location.

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, # frame      
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def connect(connection_string):
  try:
    # connect to the vehicle
    logging.info('Connecting to vehicle on: %s' % connection_string)
    vehicle = dronekit.connect(connection_string, wait_ready=False)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    return vehicle
  except Exception as e:
    logging.error("Exception caught. Most likely connection to vehicle failed.")
    logging.error(traceback.format_exc())
    return 'None'

def arm_and_takeoff(aTargetAltitude, vehicle, timeout_wait):
  """
  Arms vehicle and fly to aTargetAltitude.
  """
  logging.info("Basic pre-arm checks")
  # don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    logging.info(" Waiting for vehicle to initialise...")
    time.sleep(timeout_wait)
  logging.info("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode = dronekit.VehicleMode("GUIDED")
  vehicle.armed = True  
  #while not vehicle.armed:    
  #  logging.info(" Waiting for arming...")
  #  time.sleep(1)
  logging.info("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude)
  # check if height is safe before going anywhere
  while True:
    logging.info(" Altitude: %s"%vehicle.rangefinder.distance)
    if vehicle.rangefinder.distance>=aTargetAltitude*0.8: 
      #Trigger just below target alt.
      logging.info("Reached target altitude")
      break
    time.sleep(0.5)


def main():
  try:
    import RPi.GPIO as GPIO
    def interrupt_button_1(channel):
      if GPIO.input(BEACON_INPUT_PIN) == 0:
        #beacon found
        millis = int(round(time.time() * 1000))
        hits = 0
        for x in range(0, 39):
          if GPIO.input(BEACON_INPUT_PIN) == 0:
            hits = hits+1
          time.sleep(0.001)
        if hits > 35:
          logging.info( str(millis) + "  Signal detected, initiating Land Mode!" )
          vehicle.mode = dronekit.VehicleMode("LAND")
        #else:
        #  print("noise detected")
    setup_buttons()
  except:
    print("Button Setup failed (no GPIO?)")
  try:
    parser = argparse.ArgumentParser(
      description='Searches for beacon')
    parser.add_argument('--connect', 
              help="vehicle connection target string.")
    parser.add_argument('--log',
              help="logging level")
    args = parser.parse_args()
    if args.connect:
      connection_string = args.connect
    else:
      print("no connection specified via --connect, exiting")
      sys.exit()
    if args.log:
      logging.basicConfig(filename='search.log', level=args.log.upper())
    else:
      print('No loging level specified, using WARNING')
      logging.basicConfig(filename='search.log', level='WARNING')
    logging.info('################## Starting script log ##################')
    logging.info("System Time:" + time.strftime("%c"))
    params = load_parameters()
    vehicle = 'None'
    while vehicle == 'None':
      vehicle = connect(connection_string)
      time.sleep(params["CON_TIMEOUT"])

    vehicle.groundspeed = params["FLY_SPEED"]

    #STEP 1 check direction the drone is facing
    start = vehicle.location.global_frame
    bearing = vehicle.heading

    #STEP 2 arm and takeoff
    arm_and_takeoff(params["START_ALTITUDE"], vehicle, params["WAIT_TIMEOUT"])
    #add the interupt event here
    try:
      GPIO.add_event_detect( BEACON_INPUT_PIN, GPIO.RISING,
        callback = interrupt_button_1, bouncetime = 40 )
    except:
        print("Adding interrupt failed, drone will not auto land.")
    
    #STEP 3 half meander once in the beginning
    # go straight  
    curr = vehicle.location.global_frame
    d = distance.VincentyDistance(meters = params["MEANDER_LENGTH"])
    dest = d.destination(geopy.Point(curr.lat, curr.lon), bearing)
    drone_dest = dronekit.LocationGlobalRelative(dest.latitude,
        dest.longitude, params["FLY_ALTITUDE"])
    logging.info('Going to: %s' % drone_dest)
    #goto_position_target_global_int(drone_dest, vehicle)
    vehicle.simple_goto(drone_dest)
    time.sleep(3)
    while vehicle.groundspeed > 0.4:
      time.sleep(0.5)
    #go sideways
    curr = vehicle.location.global_frame
    d = distance.VincentyDistance(meters = params["MEANDER_WIDTH"]/2)
    dest = d.destination(geopy.Point(curr.lat, curr.lon), bearing+90)
    drone_dest = dronekit.LocationGlobalRelative(dest.latitude, 
      dest.longitude, params["FLY_ALTITUDE"])
    logging.info('Going to: %s' % drone_dest)
    #goto_position_target_global_int(drone_dest, vehicle)
    vehicle.simple_goto(drone_dest)
    time.sleep(3)
    while vehicle.groundspeed > 0.4:
      time.sleep(0.5)

    #STEP 4 calculate search path
    sign=-1
    for i in range(1,params["MEANDER_COUNT"]+1):
      if vehicle.mode.name != "GUIDED":
        logging.warning("Flight mode changed - aborting follow-me")
        break
      #go straight
      curr = vehicle.location.global_frame
      d = distance.VincentyDistance(meters = params["MEANDER_LENGTH"])
      dest = d.destination(geopy.Point(curr.lat, curr.lon), bearing)
      drone_dest = dronekit.LocationGlobalRelative(dest.latitude,
        dest.longitude, params["FLY_ALTITUDE"])
      logging.info('Going to: %s' % drone_dest)
      goto_position_target_global_int(drone_dest, vehicle)
      #vehicle.simple_goto(drone_dest)
      time.sleep(2)
      while vehicle.groundspeed > 0.4:
        time.sleep(0.5)
      #go sideways
      curr = vehicle.location.global_frame
      d = distance.VincentyDistance(meters = params["MEANDER_WIDTH"])
      dest = d.destination(geopy.Point(curr.lat, curr.lon), bearing+90*sign)
      drone_dest = dronekit.LocationGlobalRelative(dest.latitude, 
        dest.longitude, params["FLY_ALTITUDE"])
      logging.info('Going to: %s' % drone_dest)
      goto_position_target_global_int(drone_dest, vehicle)
      #vehicle.simple_goto(drone_dest)
      time.sleep(3)
      while vehicle.groundspeed > 0.4:
        time.sleep(0.5)
      sign=sign*-1
      i=i+1
    logging.info('left search mode')
    vehicle.mode = dronekit.VehicleMode('LAND')
  except Exception as e:
    logging.error("Caught exeption")
    logging.error(traceback.format_exc())
    sys.exit(1)

if __name__ == "__main__":
  main()

