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
      logging.basicConfig(filename='log-follow.log', level=args.log.upper())
    else:
      print('No loging level specified, using WARNING')
      logging.basicConfig(filename='log-follow.log', level='WARNING')
    logging.warning('###x############### Starting script log ##################')
    logging.info("System Time:" + time.strftime("%c"))
    vehicle = 'None'
    while vehicle == 'None':
      vehicle = connect(connection_string)

    #vehicle.groundspeed = SPEED #m/s
    #step1 check direction the drone is facing
    start = vehicle.location.global_frame
    bearing = vehicle.heading
    #step2 arm and takeoff
    arm_and_takeoff(3, vehicle)
    #step3 calculate search path
    d = geopy.distance.VincentyDistance(meters = 10)
    dest = d.destination(geopy.Point(start.lat, start.lon), bearing)
    drone_dest = dronekit.LocationGlobalRelative(dest.latitude, dest.longitude, 3)
    #(can be various patterns)(can be parralel to step2)
    #step4 follow the search path until signal is found (or battery is empty)
    vehicle.simple_goto(drone_dest)
    
    vehicle.mode = dronekit.VehicleMode('LAND')
  except Exception as e:
    logging.error("Caught exeption")
    logging.error(traceback.format_exc())
    sys.exit(1)

if __name__ == "__main__":
  main()

