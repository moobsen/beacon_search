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
import traceback
import sys, time
import RPi.GPIO as GPIO

BEACON_INPUT_PIN = 17 #global GPIO PIN number

def setup_buttons():
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(BEACON_INPUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def main():
  setup_buttons()
  try:
    print("Starting Signal detector")
    start_time_ms = int(round(time.time() * 1000))
    f = open('plot.dat', 'w')
    f.seek(0)
    hits=0
    i = 0
    while True:
      time.sleep(0.0005)
      i=i+1
      if GPIO.input(BEACON_INPUT_PIN) == 0:
      # NO signal
        f.write(str(i) +' 0\n')
        if hits > 0:
          hits = hits-1
      else:
      # SIGNAL
        f.write(str(i) +' 1\n')
        hits = hits+1
      now_ms = int(round(time.time() * 1000))
      #if hits == 10:
      #  print(str(now_ms-start_time_ms) + ' ms hits is 10')
      #if hits == 20:
      #  print(str(now_ms-start_time_ms) + ' ms hits is 20')
      if hits > 11:
        #print(str(now_ms-start_time_ms) + 'ms; Signal Detected')
        hits = 0
      sys.stdout.flush()
      if i > 10000:
        i=0
        f.seek(0)
  except Exception as e:
    print("Caught exeption")
    print(traceback.format_exc())
    sys.exit(1)

if __name__ == "__main__":
  main()
