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


class SearchController:
  BEACON_INPUT_PIN = 17 #GPIO PIN number in Raspberry BCM Mode
  vehicle = 'None'
  connection_string = 'None'
  start_time_ms = 0
  params = []
  
  def start_log(self):
    #start log entry
    logging.info('################## Starting script log ##################')
    logging.info("System Time:" + time.strftime("%c"))

  def load_parameters(self):
    try:
      stream = file('/home/pi/src/moobsen/beacon_search/PARAMETERS.yaml', 'r')
    except:
      import os
      stream = file(os.getcwd()+'/PARAMETERS.yaml','r')
    self.params = load(stream)
    logging.info('Loaded parameters: %s' % self.params)
    return self.params

  def goto_position_above_terrain(self, aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a location.
    """
    msg = self.vehicle.message_factory.set_position_target_global_int_encode(
      0,       # time_boot_ms (not used)
      0, 0,    # target system, target component
      mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, # frame      
      0b0000111111111000, # type_mask (only speeds enabled)
      aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
      aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
      aLocation.alt, # alt - Altitude in m, interpretation depends on GLOBAL_TERRAIN_ALT_INT
      0, # X velocity in NED frame in m/s
      0, # Y velocity in NED frame in m/s
      0, # Z velocity in NED frame in m/s
      0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
      0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    self.vehicle.send_mavlink(msg)
    self.vehicle.flush()

  def wait_until_there(self):
    time.sleep(self.params["MEANDER_MIN_TIMEOUT"])
    while self.vehicle.groundspeed > self.params["MEANDER_MIN_SPEED"]:
      time.sleep(self.params["MEANDER_MIN_SPEED_TIMEOUT"])

  def connect(self):
    try:
      # connect to the vehicle
      logging.info('Connecting to vehicle on: %s' % self.connection_string)
      print self.connection_string
      self.vehicle = dronekit.connect(self.connection_string, wait_ready=False)
    except Exception as e:
      logging.error("Exception caught. Most likely connection to vehicle failed.")
      logging.error(traceback.format_exc())
      self.vehicle = 'None'

  def arm_and_takeoff(self):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    logging.info("Basic pre-arm checks")
    # don't let the user try to arm until autopilot is ready
    while not self.vehicle.is_armable:
      logging.info(" Waiting for vehicle to initialise...")
      time.sleep(timeout_wait)
    logging.info("Arming motors")
    # Copter should arm in GUIDED mode
    self.vehicle.mode = dronekit.VehicleMode("GUIDED")
    self.vehicle.armed = True  
    logging.info("Taking off!")
    self.vehicle.simple_takeoff(self.params["TAKEOFF_ALTITUDE"])
    # check if height is safe before going anywhere
    while True:
      logging.info(" rangefinder.distance: %s"%self.vehicle.rangefinder.distance)
      current_altitude = self.vehicle.location.global_relative_frame.alt 
      if current_altitude >= self.params["TAKEOFF_ALTITUDE"]*0.9: 
        #Trigger just below target alt.
        logging.info("Reached target altitude")
        break
      time.sleep(0.5)

  def go_forward(self, forward_distance, bearing):
    """
    /vehicle/ goes for /distance/ meters in the direction of /bearing/
    """
    curr = self.vehicle.location.global_frame
    d = geopy.distance.VincentyDistance(meters = forward_distance)
    dest = d.destination(geopy.Point(curr.lat, curr.lon), bearing)
    drone_dest = dronekit.LocationGlobalRelative(dest.latitude,
      dest.longitude, self.params["FLY_ALTITUDE"])
    logging.info('Going to: %s' % drone_dest)
    self.goto_position_above_terrain(drone_dest)
    time.sleep(self.params["MEANDER_MIN_TIMEOUT"])
    while self.vehicle.groundspeed > self.params["MEANDER_MIN_SPEED"]:
      time.sleep(self.params["MEANDER_MIN_SPEED_TIMEOUT"])

  def polling_function(self):
    #TODO propper threading, this is not fully useable yet
    try:
      start_time_ms = int(round(time.time() * 1000))
      signal=0
      while True:
        time.sleep(0.0005)
        if GPIO.input(BEACON_INPUT_PIN) == 1:
          if signal > 0:
            signal = signal-1
          else:
            signal = signal+1
        now_ms = int(round(time.time() * 1000))
        if signal > 21:
          print(str(now_ms-start_time_ms) + 'ms; Signal Detected')
          signal = 0
    except Exception as e:
      logging.error(e)
      logging.error(traceback.format_exc())


  def interrupt_function(self, channel):
    if GPIO.input(BEACON_INPUT_PIN) == 0:
      #beacon found
      now_ms = int(round(time.time() * 1000))
      hits = 0
      for x in range(0, 200):
        if GPIO.input(BEACON_INPUT_PIN) == 0:
          hits = hits+1
        time.sleep(0.0005)
      if hits > 101:
        logging.info(str(now_ms-start_time_ms)+"ms; hits: "+str(hits)+" Signal detected!")
        logging.info('Landing Drone!')
        vehicle.mode = dronekit.VehicleMode("LAND")
      else:
        print(str(now_ms-start_time_ms) + "ms; hits: " + str(hits) + " ignored")

  def __init__(self, connection_string):
    start_time_ms = int(round(time.time() * 1000))
    self.start_log()
    self.connection_string = connection_string
    try:  #setup LVS receiver connection
      import RPi.GPIO as GPIO
      GPIO.setmode(GPIO.BCM)
      GPIO.setup(BEACON_INPUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    except Exception as e:
      print("LVS Setup failed (no GPIO?)")
      logging.error(e)
    #setup drone connection
    try:
      params = self.load_parameters()
      drone = self.vehicle
      while drone == 'None':
        drone = self.connect()
        time.sleep(self.params["CON_TIMEOUT"])
      self.vehicle.groundspeed = self.params["FLY_SPEED"]
      #add the interupt event here
      try:
        GPIO.add_event_detect( BEACON_INPUT_PIN, GPIO.RISING,
          callback = self.interrupt_function, bouncetime = 40 )
      except:
        logging.error("No LVS is set up, drone will not auto land.")
    except Exception as e:
      logging.error("Caught exeption")
      logging.error(traceback.format_exc())

  def search_beacon(self):
    if self.vehicle == 'None':
      logging.error("No connection to vehicle.")
      sys.exit()
    try:
      #STEP 1: check direction the drone is facing
      start = self.vehicle.location.global_frame
      bearing = self.vehicle.heading

      #STEP 2: takeoff
      self.arm_and_takeoff()
      
      #STEP 3: half meander once in the beginning
      # go straight  
      self.go_forward(self.params["MEANDER_LENGTH"], bearing)
      #go sideways
      self.go_forward(self.params["MEANDER_WIDTH"]/2, bearing+85)

      #STEP 4 calculate search path for normal meander
      sign=-1
      for i in range(1,self.params["MEANDER_COUNT"]):
        if self.vehicle.mode.name != "GUIDED":
          logging.warning("Flight mode not Guided - aborting!")
          break
        #go straight
        self.go_forward(self.params["MEANDER_LENGTH"], bearing)
        #go sideways
        self.go_forward(self.params["MEANDER_WIDTH"], bearing+85*sign)
        sign=sign*-1
        logging.info('left search mode')

      #STEP 5: land
      self.vehicle.mode = dronekit.VehicleMode('LAND')
    except Exception as e:
      logging.error("Caught exeption")
      logging.error(traceback.format_exc())
      sys.exit(1)

def main():
  #Argument Parsing
  parser = argparse.ArgumentParser(description='Find avalanche beacon with drone')
  parser.add_argument('--connect', help="vehicle connection target string.")
  parser.add_argument('--log', help="logging level")
  args = parser.parse_args()
  if args.connect:
    connection_string = args.connect
  else:
    print("No connection specified via --connect, trying 127.0.0.1:14551")
    connection_string = "127.0.0.1:14551"
  if args.log:
    logging.basicConfig(filename='search.log', level=args.log.upper())
  else:
    print('No loging level specified, using DEBUG')
    logging.basicConfig(filename='search.log', level='DEBUG')
  sc = SearchController(connection_string)
  sc.search_beacon()
  
if __name__ == "__main__":
  main()

