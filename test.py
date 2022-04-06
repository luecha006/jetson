# from hashlib import new
from itertools import count
import sys
from tkinter.tix import Tree
sys.path.append('/home/jetson/Desktop/project/keras_imagenet')

import cv2
import numpy as np
from PIL import Image, ImageOps
from predict_image import main
from threading import Thread
import busio
import board
import time
import adafruit_amg88xx
from enum import Flag
import imp
from pytools import F

from read_sensor import thermal_msg
from display_sound import displaysound_mp3
from  stackImages import stackImages
from overlayImageAndText import overlay, masked_msg, introduction_hightemp_msg, introduction_unmasked_msg, nobody_msg, thermal_msg, notpass_msg

from trt_mtcnn import main_detect

TEMP_CARIBRATION = 7.0

#  สร้าง Thread ที่ return ค่าได้
class amg8833 :
    device = None   # device data
    i2c_bus = None

    def __init__ ( self, addr ) :
        # init I2C bus
        self.i2c_bus = busio.I2C(board.SCL, board.SDA)

        # init AMG8833
        try:
            self.device = adafruit_amg88xx.AMG88XX(self.i2c_bus, addr=addr)
            # wait a bit sensor initialize
            time.sleep(.1)
        except ValueError as e:
            print(e)
            self.device = None

    def __del__ ( self ) :
        if self.i2c_bus != None:
            self.i2c_bus.deinit()

    # get temperature
    def get_tempreture(self) :
        if self.device == None:
            return None, None
        else:
            return self.device.pixels, self.device.temperature

def getTemp(pixels):
    if pixels == None:
            # pixels == None means no device avaiable
        temp = 0
    else:
        # find max temprature in 8X8 then add caribiration value
        temp =  max(max(pixels)) + TEMP_CARIBRATION
        
    return temp

isDeteced = True
def delay_detect_frame(image):
    value_detect_face = main_detect(image)
    return value_detect_face


imag1_display = []
imag2_display = []
imagToPredict = []
isPredicted = True
isFirstPredictedImag = False
isCapture = False
isDeteced = False
isFirstPred = False
isFirstNumberPredicted = 1
number_pre = 1
number_dect = 1

def opencamera():
    sensor = amg8833(0x69)
    path = 'imageUser.png'
    path2 = '/home/jetson/Documents/dataset_test_facemask/test_with_mask/dataset/resize_cdvfdvxz.jpg'
    global temp
    global maskPattern
    global imag1_display
    global imag2_display
    global isPredicted
    global isFirstPredictedImag
    global isCapture
    global imagToPredict
    global number_pre
    global number_dect
    global isDeteced
    global isFirstNumberPredicted
    
    isFirstCaptureImag = False
    
    # เปิดกล้องอ่านภาพ
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    # อ่านไฟล์ตรวจจับใบหน้า
    # face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    while (cap.isOpened()):
        check, frame = cap.read()
        
        imag1_display = []
        imageUser = cv2.imread(path)
        imageUser = cv2.resize(imageUser, (650, 720))
        
        if check == True:
            image_detect = frame[100:-100, 80:-80]
            image_msg = cv2.resize(frame, (650, 720))
            image_detect = cv2.resize(image_detect, (650, 720))
            image_detect_test = cv2.resize(frame, (650, 720))
            
            # overlay frame and image
            frame_overlay = overlay(image_msg)
            
            if (isDeteced == True) and (isPredicted == True) :
                isDeteced = False
                value_detect_face = main_detect(image_detect_test)
                
            else:
                value_detect_face = 0
            
            
            if value_detect_face == 0:
                image_nobody = nobody_msg(frame_overlay)
                imag1_display.append(image_nobody)
                
            # if cv2.waitKey(1) & 0xFF == ord("e"):
            elif value_detect_face == 1 :
                
                isPredicted = False
                isCapture = True
                
                # crop image to predict
                
                # using sensor tempreture
                temperature_database = 37.0
                pixels, unit_temp = sensor.get_tempreture()
                temp = getTemp(pixels)
                # temp = 36.5
                print('temp ',temp)
                
                # predict imagee
                # cv2.imshow('image ',image_msg)
                maskPattern = main(image_detect_test)
                print('maskPattern is ',maskPattern)
                
                if(temp < 33.0):
                    image = notpass_msg(image_msg)
                    image = thermal_msg(image, temp)
                
                elif((temp <= temperature_database) and (maskPattern == 'w')):
                    image = masked_msg(image_msg)
                    image = thermal_msg(image, temp)
                
                elif ((temp > temperature_database) and (maskPattern == 'w')):
                    image = notpass_msg(image_msg)
                    image = thermal_msg(image, temp)
                    image = introduction_hightemp_msg (image, maskPattern)
                    
                elif((temp > temperature_database) and ((maskPattern == 'm') or (maskPattern == 'o'))):
                    image = notpass_msg(image_msg)
                    image = thermal_msg(image, temp)
                    image = introduction_unmasked_msg (image)
                    image = introduction_hightemp_msg (image, maskPattern)
                    
                elif((temp <= temperature_database) and ((maskPattern == 'm') or (maskPattern == 'o'))):
                    image = notpass_msg(image_msg)
                    image = thermal_msg(image, temp)
                    image = introduction_unmasked_msg (image)
                    
                imag1_display.append(frame_overlay)
                imag1_display.append(image)
                imag2_display = image
    
                # thr_sound = ThreadWithReturnValue(target=displaysound_mp3, args=(temp, maskPattern))
                # thr_sound.start()
                
                # displaysound_mp3(temp, maskPattern)
                # print('maskPattern ', maskPattern, ',temp ',temp)
                
                isPredicted = True
                isCapture = False
                value_detect_face = 0
                if(isFirstCaptureImag == False):
                    isFirstCaptureImag = True
                    
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            
            # cv2.imshow('frame_overlay ',frame_overlay)
            if(isFirstCaptureImag == True):
                if(isCapture == True):
                    print('isCapture is True')
                    StackedImages = stackImages(([imag1_display[0], imag1_display[1]]), 0.6)
                    cv2.imshow("PROJECT-C10 GUI CAMERA", StackedImages)
                else:
                    print('isCapture is False')
                    StackedImages = stackImages(([imag1_display[0], imag2_display]), 0.6)
                    cv2.imshow("PROJECT-C10 GUI CAMERA", StackedImages)
            else:
                print('isFirstCaptureImag is False')
                StackedImages = stackImages(([imag1_display[0], imageUser]), 0.6)
                cv2.imshow("PROJECT-C10 GUI CAMERA", StackedImages)
        else:
            print("Unable to open camera")
            break
    
    cap.release()
    cv2.destroyAllWindows()  # คืนทรัพยากรณ์ให้กับระบบ
    
# คลาสสร้าง object ทำงานแบบ Real-Time
class ThreadWithReturnValue(Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, *, daemon=None):
        Thread.__init__(self, group, target, name, args, kwargs, daemon=daemon)

        self._return = None

    def run(self):
        if self._target is not None:
            self._return = self._target(*self._args, **self._kwargs)

    def join(self):
        Thread.join(self)
        return self._return
    

def gstreamer_pipeline(
    capture_width=800,
    capture_height=800,
    display_width=650,
    display_height=720,
    framerate=60,
    flip_method=2,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def countdown():
    global isDeteced
    global isPredicted
    global isFirstPred
    while True:
        t = 2
        while t:
            # print('isDeteced', isDeteced)
            mins, secs = divmod(t, 60)
            timer = '{:02d}:{:02d}'.format(mins, secs)
            print(timer, end="\r")
            time.sleep(1)
            t -= 1
        if(isPredicted != False):
            if(isDeteced != True):
                isDeteced = True

if __name__=='__main__':
    thr_countdown = ThreadWithReturnValue(target=countdown)
    thr_countdown.start()
    opencamera()
    # thr_camera = ThreadWithReturnValue(target=opencamera)
    # thr_camera.start()
    
    
    