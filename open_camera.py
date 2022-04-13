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
import json
import requests

from read_sensor import thermal_msg
from display_sound import displaysound_mp3
from  stackImages import stackImages
from overlayImageAndText import overlay, masked_msg, introduction_hightemp_msg, introduction_unmasked_msg, nobody_msg, thermal_msg, notpass_msg, writeTextToImageResult

from trt_mtcnn import main_detect

save_maskpattern_api_url = "http://35.213.141.41:8080/facemask/savemaskpattern"
extract_temperature_api_url = "http://35.213.141.41:8080/temperature/extract"

TEMP_CARIBRATION = 6.0

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

def save_maskpattern_api(maskpattern, temperature):
    from datetime import datetime

    datenow = datetime.now()
    time = datenow.strftime("%H:%M:%S")
    date = datenow.strftime("%d/%m/%Y")
    
    dataToSaveMaskPattern = { "date": date, "time": time, "maskpattern": maskpattern, 'temperature': temperature}
    response = requests.post(save_maskpattern_api_url, json=dataToSaveMaskPattern)
    
def extract_temperature_api():
    response = requests.get(extract_temperature_api_url)
    # print('temperature: ',response.json())
    return response.json();

def recordTheResults(image, maskpattern, temperature):
    from datetime import datetime
    import os
    
    if maskpattern == 'w':
        maskpattern = 'masked'
    elif maskpattern == 'm':
        maskpattern = 'wearing the wrong way'
    elif maskpattern == 'o':
        maskpattern = 'no mask'
    
    datenow = datetime.now()
    time = datenow.strftime("%H:%M:%S")
    date = datenow.strftime("%d-%m-%Y")
    
    directory = '/home/jetson/predict_model_jetson/results_predict_image'
    filename = time+'_'+date+'_maskpattern-is-'+maskpattern+'_temperature-is-'+str(temperature)+'.jpg'
    
    text1 = 'Date: '+date+', Time: '+time
    text2 = 'maskpattern is '+maskpattern
    text3 = 'temperature is '+str(temperature)
    
    imageResult = writeTextToImageResult(image, text1, text2, text3)
    
    cv2.imwrite(os.path.join(directory , filename), imageResult)
    
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
count_pre = 0

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
    global count_pre
    
    isFirstCaptureImag = False
    
    dispW=800
    dispH=720
    flip=2
    # camSet='nvarguscamerasrc sensor-id=0 tnr-mode=2 tnr-strength=1 saturation=1 ee-mode=1 ee-strength=0 gainrange=16 aeantibanding=2  exposurecompensation=0 wbmode=4 !  video/x-raw(memory:NVMM), width=800, height=720, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! videobalance contrast=1.5 brightness=-0.1 saturation=1.5 ! appsink'
    # camSet='nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=21/1,format=NV12 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
    
    # เปิดกล้องอ่านภาพ
    # cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    cap = cv2.VideoCapture('/dev/video1')
    # cap=cv2.VideoCapture(camSet)
    
    while (cap.isOpened()):
        check, frame = cap.read()
        frame = cv2.resize(frame, (650,720))
        if(isFirstPredictedImag == True) and (isFirstNumberPredicted <= 1):
            print('sleep')
            time.sleep(10)
            isFirstNumberPredicted += 1
            check = False
            frame = []
            print('continue')
            continue
        
        imag1_display = []
        imageUser = cv2.imread(path)
        imageUser = cv2.resize(imageUser, (650, 720))
        
        if check == True:
            image_detect = frame[100:-100, 80:-80]
            image_predict = image_detect
            # image_predict = frame[140:-130, 80:-80]
            image_msg = cv2.resize(image_detect, (650, 720))
            image_detect = cv2.resize(image_detect, (650, 720))
            image_write_result = image_detect
            
            # overlay frame and image
            frame_overlay = overlay(image_detect)
            
            if (isDeteced == True) and (isPredicted == True) :
                isDeteced = False
                value_detect_face = main_detect(image_detect)
                
            else:
                value_detect_face = 0
            
            if value_detect_face == 0:
                image_nobody = nobody_msg(frame_overlay)
                imag1_display.append(image_nobody)
                
            # if cv2.waitKey(1) & 0xFF == ord("e"):
            elif value_detect_face == 1 :
                isPredicted = False
                isCapture = True
                            
                # using sensor tempreture
                pixels, unit_temp = sensor.get_tempreture()
                temp = getTemp(pixels)
                # temp = 37.9
                print('temp ',temp)
                
                # get tempreture on api
                # temperature_database = 37.5
                temperature_database = extract_temperature_api() #ดึงอุณหภูมิจาก data base
                # print('temperature_database ',temperature_database)
                
                # predict imagee
                maskPattern = main(image_predict)
                    
                print('maskPattern is ',maskPattern)
                
                # สร้างรูปภาพ
                if(temp < 30.0):
                    image = notpass_msg(image_msg)
                    image = thermal_msg(image, temp, temperature_database)
                
                elif((temp <= temperature_database) and (maskPattern == 'w')):
                    image = masked_msg(image_msg)
                    image = thermal_msg(image, temp, temperature_database)
                
                elif ((temp > temperature_database) and (maskPattern == 'w')):
                    image = notpass_msg(image_msg)
                    image = thermal_msg(image, temp, temperature_database)
                    image = introduction_hightemp_msg (image, maskPattern)
                    
                elif((temp > temperature_database) and ((maskPattern == 'm') or (maskPattern == 'o'))):
                    image = notpass_msg(image_msg)
                    image = thermal_msg(image, temp, temperature_database)
                    image = introduction_unmasked_msg (image)
                    image = introduction_hightemp_msg (image, maskPattern)
                    
                elif((temp <= temperature_database) and ((maskPattern == 'm') or (maskPattern == 'o'))):
                    image = notpass_msg(image_msg)
                    image = thermal_msg(image, temp, temperature_database)
                    image = introduction_unmasked_msg (image)
                    
                imag1_display.append(frame_overlay)
                imag1_display.append(image)
                imag2_display = image
    
                displaysound_mp3(temp, maskPattern, temperature_database)
                
                save_maskpattern_api(maskPattern, temp)
                # recordTheResults(image_write_result, maskPattern, temp)   #สำหรับทดลองเพื่อเก็บผลลัพธ์
                
                isPredicted = True
                isCapture = False
                value_detect_face = 0
                count_pre += 1
                if (isFirstPredictedImag != True):
                    isFirstPredictedImag = True
                
                if(isFirstCaptureImag == False):
                    isFirstCaptureImag = True
                    
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            
            if(isFirstCaptureImag == True):
                if(isCapture == True):
                    StackedImages = stackImages(([imag1_display[0], imag1_display[1]]), 0.6)
                    cv2.imshow("PROJECT-C10 GUI CAMERA", StackedImages)
                else:
                    StackedImages = stackImages(([imag1_display[0], imag2_display]), 0.6)
                    cv2.imshow("PROJECT-C10 GUI CAMERA", StackedImages)
            else:
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
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=21,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=0 tnr-mode=2 tnr-strength=1 saturation=1 ee-mode=1 ee-strength=0 gainrange=16 aeantibanding=2  exposurecompensation=0 wbmode=0 ! "
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

# def gstreamer_pipeline(
#     capture_width=800,
#     capture_height=720,
#     display_width=800,
#     display_height=720,
#     framerate=60,
#     flip_method=2,
# ):
#     return (
#         "nvarguscamerasrc ! "
#         "video/x-raw(memory:NVMM), "
#         "width=(int)%d, height=(int)%d, "
#         "format=(string)NV12, framerate=(fraction)%d/1 ! "
#         "nvvidconv flip-method=%d ! "
#         "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
#         "videoconvert ! "
#         "video/x-raw, format=(string)BGR ! appsink"
#         % (
#             capture_width,
#             capture_height,
#             framerate,
#             flip_method,
#             display_width,
#             display_height,
#         )
#     )

def countdown():
    global isDeteced
    global isPredicted
    global isFirstPred
    while True:
        t = 5
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
    
    #Test
    # displaysound_mp3(37.5, 'w')
    
    # save_maskpattern_api('w', 35.6)
    # extract_temperature_api()
    
    
    
    
    
