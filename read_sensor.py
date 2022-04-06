from PIL import Image, ImageOps
import numpy as np
import time

import busio
import board

import adafruit_amg88xx

from display_sound import displaysound_mp3

# from display_sound import playsound

# cariblation temperature
TEMP_CARIBRATION = 7.0
TEMP_THRESHOLD = 36.0

#===================
# class for AMG8833
# class amg8833 :
#     device = None   # device data
#     i2c_bus = None

#     def __init__ ( self, addr ) :
#         # init I2C bus
#         self.i2c_bus = busio.I2C(board.SCL, board.SDA)

#         # init AMG8833
#         try:
#             self.device = adafruit_amg88xx.AMG88XX(self.i2c_bus, addr=addr)
#             # wait a bit sensor initialize
#             time.sleep(.1)
#         except ValueError as e:
#             print(e)
#             self.device = None

#     def __del__ ( self ) :
#         if self.i2c_bus != None:
#             self.i2c_bus.deinit()

#     # get temperature
#     def get_tempreture(self) :
#         if self.device == None:
#             return None, None
#         else:
#             return self.device.pixels, self.device.temperature
        
def thermal_msg(pixels):
    if pixels == None:
        # pixels == None means no device avaiable
        temp = 0
    else:
        # find max temprature in 8X8 then add caribiration value
        temp =  max(max(pixels)) + TEMP_CARIBRATION

    # show tempreture
    # if temp == 0:
    #     print('No thermal sensor available.')                
    # elif temp > TEMP_THRESHOLD:
    #     # print('tempreture max. :',temp)
    #     displaysound_mp3(temp, 'o')
    # else:
    #     displaysound_mp3(temp, 'w')
        
    return temp


# if __name__=='__main__':
#     sensor = amg8833(0x69)
#     while True :
#         pixels, unit_temp = sensor.get_tempreture()
#         thermal_msg(pixels)
        