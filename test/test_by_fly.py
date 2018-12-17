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
import context
import dronekit
import logging
import traceback
import time
import sys
import beacon_search

def main():
  try:
    sc=beacon_search.SearchController('127.0.0.1:14551')
    sc.start_log
    logging.basicConfig(filename='test_by_fly.log', level='DEBUG')
    sc.arm_and_takeoff()
    
    #STEP1 go to start
    logging.info('Going to start')
    start_location = dronekit.LocationGlobalRelative(53.476033, 9.929890, 1)
    sc.goto_position_above_terrain(start_location)
    sc.wait_until_there()

    #STEP2 go to finish
    logging.info('Going to finish')
    finish_location = dronekit.LocationGlobalRelative(53.476038, 9.929576, 1)
    sc.goto_position_above_terrain(finish_location)
    sc.wait_until_there()

    #STEP4 land
    logging.info('landing')
    sc.vehicle.mode = dronekit.VehicleMode("LAND")

  except:
    logging.error("Caught exeption")
    logging.error(traceback.format_exc())
    sys.exit(1)

if __name__ == "__main__":
  main()
