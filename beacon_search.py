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
import gps
import threading
import time
import sys
import math
import logging
import geopy
import geopy.distance
import RPi.GPIO as GPIO

#################### PARAMETERS ################################################
#GPIOs (using BCM style)
BEACON_INPUT_PIN  = 17
#other parameters
START_ALTITUDE = 6# in meters
FLY_ALTITUDE = 3  # in meters
FLY_SPEED = 10 # in meters/second
MEANDER_DISTANCE = 5 #in meters
MEANDER_COUNT = 4
SEARCH_ANGLE = 30 #in degrees

def setup_buttons():
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(BEACON_INPUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def connect(connection_string):
  try:
    # connect to the vehicle
    logging.info('Connecting to vehicle on: %s' % connection_string)
    vehicle = dronekit.connect(connection_string, wait_ready=True)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    return vehicle
  except Exception as e:
    logging.error("Exception caught. Most likely connection to vehicle failed.")
    logging.error(traceback.format_exc())
    return 'None'

def arm_and_takeoff(aTargetAltitude, vehicle):
  """
  Arms vehicle and fly to aTargetAltitude.
  """
  logging.info("Basic pre-arm checks")
  # don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    logging.info(" Waiting for vehicle to initialise...")
    time.sleep(1)
  logging.info("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode = dronekit.VehicleMode("GUIDED")
  vehicle.armed = True  
  while not vehicle.armed:    
    logging.info(" Waiting for arming...")
    time.sleep(1)
  logging.info("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude)
  # check if height is safe before going anywhere
  while True:
    logging.info(" Altitude: %s"%vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.7: 
      #Trigger just below target alt.
      logging.info("Reached target altitude")
      break
    time.sleep(1)


def main():
  def interrupt_button_1(channel):
    if GPIO.input(BEACON_INPUT_PIN) == 1: 
      #beacon found
      logging.warning("Beacon found, landing!")
      vehicle.mode = dronekit.VehicleMode("LAND")

  setup_buttons()
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
      logging.basicConfig(filename='log-follow.log', level='WARNING')
    logging.warning('################## Starting script log ##################')
    logging.info("System Time:" + time.strftime("%c"))
    vehicle = 'None'
    while vehicle == 'None':
      vehicle = connect(connection_string)

    #vehicle.groundspeed = SPEED #m/s
    #step1 check direction the drone is facing
    start = vehicle.location.global_frame
    bearing = vehicle.heading
    #step2 arm and takeoff
    arm_and_takeoff(START_ALTITUDE, vehicle)
    #add the interupt event here
    GPIO.add_event_detect(BEACON_INPUT_PIN, GPIO.RISING,
      callback = interrupt_button_1, bouncetime = 100)

    #step3 calculate search path
    sign=1
    for i in range(1,MEANDER_COUNT):
      if vehicle.mode.name != "GUIDED":
        logging.warning("Flight mode changed - aborting follow-me")
        break
      #go straight
      curr = vehicle.location.global_frame
      d = geopy.distance.VincentyDistance(meters = MEANDER_DISTANCE)
      dest = d.destination(geopy.Point(curr.lat, curr.lon), bearing)
      drone_dest = dronekit.LocationGlobalRelative(dest.latitude,
        dest.longitude, FLY_ALTITUDE)
      logging.info('Going to: %s' % drone_dest)
      vehicle.simple_goto(drone_dest)
      time.sleep(3)
      while vehicle.groundspeed > 0.5:
        time.sleep(1)
      #go sideways
      curr = vehicle.location.global_frame
      tan_y = math.tan(math.radians(SEARCH_ANGLE/2))
      d = geopy.distance.VincentyDistance(
        meters = 2*i*MEANDER_DISTANCE*tan_y-tan_y*MEANDER_DISTANCE )
      dest = d.destination(geopy.Point(curr.lat, curr.lon), bearing+90*sign)
      drone_dest = dronekit.LocationGlobalRelative(dest.latitude, 
        dest.longitude, FLY_ALTITUDE)
      logging.info('Going to: %s' % drone_dest)
      vehicle.simple_goto(drone_dest)
      time.sleep(3)
      while vehicle.groundspeed > 0.5:
        time.sleep(1)
      sign=sign*-1
      i=i+1

    vehicle.mode = dronekit.VehicleMode('LAND')
  except Exception as e:
    logging.error("Caught exeption")
    logging.error(traceback.format_exc())
    sys.exit(1)

if __name__ == "__main__":
  main()

