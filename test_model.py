import cv2
import numpy as np
from PIL import Image, ImageOps
from pytools import T
from predict_image import main
from threading import Thread
import time

from overlayImageAndText import writeTextToImageResult


from trt_mtcnn import main_detect

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

def opencamera():
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
    
    dispW=800
    dispH=800
    flip=2
    # camSet='nvarguscamerasrc sensor-id=0 tnr-mode=2 tnr-strength=1 saturation=1 ee-mode=1 ee-strength=0 gainrange=16 aeantibanding=2  exposurecompensation=0 wbmode=4 !  video/x-raw(memory:NVMM), width=800, height=720, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! videobalance contrast=1.5 brightness=-0.1 saturation=1.5 ! appsink'
    camSet='nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1280, height=750, framerate=21/1,format=NV12 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

    # เปิดกล้องอ่านภาพ
    # cap = cv2.VideoCapture(camSet)
    cap=cv2.VideoCapture('/dev/video1')
    # cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    # อ่านไฟล์ตรวจจับใบหน้า
    # face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    while (cap.isOpened()):
        check, frame = cap.read()
        frame = cv2.resize(frame, (650,720))
        image_predict = frame[120:-110, 80:-80]
        image_detect = frame[100:-100, 80:-80]
        imageresule = image_detect
        if check == True:
            
            if cv2.waitKey(1) & 0xFF == ord("e"):
                
                maskPattern = main(image_detect)
                print('maskPattern is ',maskPattern)
                
                # recordTheResults(imageresule, 'm' , 36.6)
                        
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            print("Unable to open camera")
            break
        cv2.imshow('frame_overlay ',image_detect)
    
    cap.release()
    cv2.destroyAllWindows()  # คืนทรัพยากรณ์ให้กับระบบ
    
# คลาสสร้าง object ทำงานแบบ Real-Time
    

def gstreamer_pipeline(
    capture_width=800,
    capture_height=720,
    display_width=800,
    display_height=720,
    framerate=60,
    flip_method=2,
):
    return (
        "nvarguscamerasrc !"
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR !appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )
  
  
if __name__=='__main__':
    opencamera()
