#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    Copyright {2020} {Bluebird Mountain | Moritz Obermeier}

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
import time
import sys
import math
import logging
import geopy
try:
    import RPi.GPIO as GPIO #works only on a Raspberry Pi
except:
    noRPi = True #alternative for simulation on other computer
from geopy import distance 
from pymavlink import mavutil
from yaml import safe_load

class SearchController:
  #class attributes
  BEACON_INPUT_PIN = 17 #GPIO PIN number in Raspberry BCM Mode
  vehicle = 'None'
  connection_string = 'None'
  start_time_ms = 0
  signal_detection=True
  params = []

  def load_parameters(self):
    try:
      stream = open('/home/pi/src/moobsen/beacon_search/PARAMETERS.yaml', 'r')
    except:
      import os
      stream = open(os.getcwd()+'/PARAMETERS.yaml','r')
    self.params = safe_load(stream)
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
      self.vehicle = dronekit.connect(self.connection_string, wait_ready=False)
    except Exception as e:
      logging.error("Exception caught. Most likely connection to vehicle failed.")
      logging.error(traceback.format_exc())
      self.vehicle = 'None'

  def arm_and_takeoff(self):
    """
    Arms vehicle and takes off to TAKEOFF_ALTITUDE
    FAILS IF VEHICLE IS ALREADY ARMED
    """
    logging.info("Basic pre-arm routine")
    logging.info("Arming motors in Stabilize to losen them")
    self.vehicle.mode = dronekit.VehicleMode("STABILIZE")
    self.vehicle.armed = True
    time.sleep(self.params["WAIT_ARM_TIMEOUT"])
    # For beacon search we need Guided Mode
    logging.info("Trying to change to Guided")
    self.vehicle.mode = dronekit.VehicleMode("GUIDED")
    start_time_ms = int(round(time.time() * 1000))
    while self.vehicle.mode != "GUIDED":
      self.vehicle.mode = dronekit.VehicleMode("GUIDED")
      time.sleep(self.params["WAIT_ARM_TIMEOUT"])
    while self.vehicle.armed is False:
      self.vehicle.armed = True
      now_ms = int(round(time.time() * 1000))
      logging.info(str( (now_ms-self.start_time_ms)/1000 ) + \
      "s: Waiting for drone to arm in guided... bad GPS?")
      time.sleep(self.params["WAIT_ARM_TIMEOUT"])  
    logging.info("Taking off!")
    self.vehicle.simple_takeoff(self.params["TAKEOFF_ALTITUDE"])
    # check if height is safe before going anywhere
    while True:
      current_altitude = self.vehicle.location.global_relative_frame.alt 
      logging.info(" rangefinder.distance: %s"%current_altitude)
      if current_altitude >= self.params["TAKEOFF_ALTITUDE"]*0.9: 
        #Trigger just below target alt.
        logging.info("Reached target altitude")
        break
      time.sleep(0.5)

  def go_forward(self, forward_distance, bearing):
    curr = self.vehicle.location.global_frame
    d = distance.distance(meters = forward_distance)
    dest = d.destination(geopy.Point(curr.lat, curr.lon), bearing)
    drone_dest = dronekit.LocationGlobalRelative(dest.latitude,
      dest.longitude, self.params["FLY_ALTITUDE"])
    logging.info('Going to: %s' % drone_dest)
    #self.goto_position_above_terrain(drone_dest)
    self.vehicle.simple_goto(drone_dest)
    time.sleep(self.params["MEANDER_MIN_TIMEOUT"])
    while self.vehicle.groundspeed > self.params["MEANDER_MIN_SPEED"]:
      time.sleep(self.params["MEANDER_MIN_SPEED_TIMEOUT"])

  def interrupt_function(self, channel):
    now_ms = int(round(time.time() * 1000))
    hits = 0
    for x in range(0, 140):
      if self.GPIO.input(self.BEACON_INPUT_PIN) == 1:
        hits = hits+1
      #else:
      #  hits = hits-1
      time.sleep(0.0005) #lowest on raspi
    if hits > 85:
      logging.info(str(now_ms-self.start_time_ms)+"ms; hits: "+str(hits)+" Signal detected!")
      logging.info('Landing Drone!')
      self.vehicle.mode = dronekit.VehicleMode("LAND")
      GPIO.remove_event_detect(self.BEACON_INPUT_PIN)
    else:
      logging.debug(str(now_ms-self.start_time_ms) + "ms; hits: " + str(hits) + " Signal ignored")

  def send_ned_velocity(self, velocity_x=0, velocity_y=0, velocity_z=0, duration=0):
    #Move vehicle in direction based on specified velocity vectors
    msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
      0,       # time_boot_ms (not used)
      0, 0,    # target system, target component
      mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
      0b0000111111000111, # type_mask (only speeds enabled)
      0, 0, 0, # x, y, z positions (not used)
      velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
      0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
      0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        self.vehicle.send_mavlink(msg)
        time.sleep(1)

  def __init__(self, connection_string, signal_detection):
    logging.info('#################### Beginning SC init ########################')
    self.start_time_ms = int(round(time.time() * 1000))
    self.connection_string = connection_string #needed to overwrite default of the class for all methods
    self.signal_detection = signal_detection #needed to overwrite default of the class for all methods

    # Setup drone connection
    try:
      params = self.load_parameters()
      drone = self.vehicle
      while drone == 'None':
        drone = self.connect()
        time.sleep(self.params["CON_TIMEOUT"])
      self.vehicle.groundspeed = self.params["FLY_SPEED"]
    except Exception as e:
      logging.error("Caught exeption")
      logging.error(traceback.format_exc())

    # Setup signal detection
    if self.signal_detection == True:
      if noRPi == False: #if rpi.gpio was imported sucessfully
        try: 
          #setup LVS receiver connection
          #import RPi.GPIO as GPIO
          self.GPIO = GPIO
          self.GPIO.setmode(GPIO.BCM)
          self.GPIO.setup(self.BEACON_INPUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        except Exception as e:
          logging.error("Signal detection setup failed (no GPIO?)")
          logging.error(e)
      else:
        logging.info('Signal detection not active! Signal detection is not possible. Either not working on a raspberry pi or python package <rpi.gpio> is not installed')
    else:
      logging.info("Signal detection not active!")

  def search_beacon(self, search_pattern='triang'): 
    if self.vehicle == 'None':
      logging.error("No connection to vehicle.")
      sys.exit()
    try:
      #STEP 1: check direction the drone is facing
      start = self.vehicle.location.global_frame
      bearing = self.vehicle.heading

      #STEP 2: takeoff (optionally activate search for beacon) 
      self.arm_and_takeoff()
      if self.signal_detection == True:
        if noRPi == False:
          GPIO.add_event_detect(self.BEACON_INPUT_PIN, GPIO.RISING, callback = self.interrupt_function, bouncetime = 70 )
        else:
          logging.info ('Signal detection not active! Signal detection is not possible. Either not working on a raspberry pi or python package <rpi.gpio> is not installed')

      #STEP 3: Start a search pattern
      #STEP 3.a: Triangular opening meander (directed search)
      if search_pattern == 'triang': # meander in an opening triangular
        #half meander once in the beginning
        #go straight
        self.go_forward(self.params["MEANDER_LENGTH"], bearing)
        #go sideways
        self.go_forward(self.params["MEANDER_WIDTH"]/2, bearing+85)

        #calculate search path for normal meander
        sign=-1
        for i in range(1,self.params["MEANDER_COUNT"]):
          if self.vehicle.mode.name != "GUIDED":
            logging.warning("Flight mode not Guided - aborting!")
            break
          #go straight
          print('go forward')
          self.go_forward(self.params["MEANDER_LENGTH"], bearing)
          print('went forward')
          #go sideways
          print('go sideways')
          self.go_forward(self.params["MEANDER_WIDTH"], bearing+85*sign)
          print('went sideways')
          sign=sign*-1

        logging.info('left search mode')

      #STEP 3.b: Archimedean Spiral (open search in all directions)
      if search_pattern == 'spiral': # move in an spiral
        start_s = int(round(time.time()))
        alpha = self.params["SPIRAL_ALPHA"] # Starting angle
        dist_control = self.params["SPIRAL_DIST_CONTROL"] # Factor that controls the distance between loops of spiral
        speed = self.params["SPIRAL_SPEED"] # Speed for spiral flight
        max_flight_time = self.params["MAX_FLIGHT_TIME"]
        dur = 1 # !=0 to avoid devision by 0 and 1 for a quite neat first spiral loop
        while dur < max_flight_time:
          if self.vehicle.mode.name != "GUIDED":
            logging.warning("Flight mode not Guided - aborting!")
            break
          x = math.sin(alpha)*speed #alpha controles the direction, speed controls the total speed
          y = math.cos(alpha)*speed
          self.send_ned_velocity(x,y,0,1)
          alpha += 1/math.sqrt(dur)*dist_control # change the angle in a way that an archimedean spiral is formed
          dur = 5 + int(round(time.time()))-start_s # 1
          print('Spiral search: Flight time: ' +str(dur) + ' s')

        logging.info('left search mode') 

      else:
        logging.info('pattern '+ str(search_pattern) +' not found, search did not start')

      #STEP 4: land
      self.vehicle.mode = dronekit.VehicleMode('LAND')

    except Exception as e:
      logging.error("Caught exeption")
      logging.error(traceback.format_exc())
      sys.exit(1)


def main():
  logging.basicConfig(
                      filename='search.log',
                      format='%(asctime)s %(levelname)-8s %(message)s',
                      level=logging.DEBUG,
                      datefmt='%Y-%m-%d %H:%M:%S')

  #Argument Parsing
  parser = argparse.ArgumentParser(description='Find avalanche beacon with drone')
  parser.add_argument('--connect', help="vehicle connection target string.")
  parser.add_argument('--log', help="logging level")
  parser.add_argument('--nosearch', help="no search for beacon, only fly the search pattern",
                      action="store_true")
  parser.add_argument('--searchpattern', help="Optional: choose a search pattern (e.g. triang, sprial)",
                      default="triang")
  args = parser.parse_args()

  if args.log:
    logging.basicConfig(level=args.log.upper())

  if args.connect:
    connection_string = args.connect
  else:
    logging.info("No connection specified via --connect, trying 127.0.0.1:14551")
    connection_string = "127.0.0.1:14551"
  if args.searchpattern:
    search_pattern = args.searchpattern

  #Creating SearchController Object
  if args.nosearch:
    sc = SearchController(connection_string, signal_detection = False)
  else:
    sc = SearchController(connection_string, signal_detection = True)

  #Starting SearchCotroller Instance
  sc.search_beacon(search_pattern)
  sys.exit(0)

#Only excecute main() if this script is running as __main__ (i.e. directly executed by python, not by python script)
if __name__ == "__main__":
  main()
