# -*- coding: utf-8 -*-
"""
@file get_all_data.py
@brief This is a demo to retrieve all matrix data. Running this code will print the matrix data.
@copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT License (MIT)
@author [tangjie](jie.tang@dfrobot.com)
@version  V1.0
@date  2024-09-9
@url https://github.com/DFRobot/DFRobot_TOF
"""

from __future__ import print_function
import sys
import os
sys.path.append("../")
import time

from DFRobot_tof import *

ADDRESS = 0x30

tof = DFRobot_TOF(ADDRESS)

def setup():
  while tof.begin() != 0:
    print("begin error!!!!")
    time.sleep(1)
  while tof.get_all_data_config(4,0) != 0:
    print("init error")
    time.sleep(1)
    

def loop():
	data = tof.get_all_data()
	for index, value in enumerate(data):
		print(value, end=' ')
		if (index + 1) % 4 == 0:
			print()  # 换行
	print("-------------------")
	time.sleep(1)
  
    
if __name__ == "__main__":
  setup()
  while True:
    loop()