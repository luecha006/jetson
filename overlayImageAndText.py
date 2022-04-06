import cv2
from PIL import Image, ImageOps
import numpy as np

#messages
MASKEDANDLOWTEMP_MSG = 'Pass.'
MASKEDANDHIGHTEMP_MSG = 'Not pass.'
INTRODUCTION_UNMASKEDMSG_MSG = 'Please wear a mask to cover your mouth and nose.'
INTRODUCTION_HIGHTEMP_MSG = 'Temperature is overdue, please scan again.'
NOBODY_MSG = 'Please set your face on the frame.'

font = cv2.FONT_HERSHEY_TRIPLEX

# cariblation temperature
TEMP_CARIBRATION = 7.0
# TEMP_THRESHOLD = 37.0

# overlay frame PNG with alpha and video frame
def overlay(frame_overlay):
    # frame = cv2.resize(frame_overlay, (650, 720))
    frame = cv2.rectangle(frame_overlay,(100,80),(550,640),(0,255,0),3)
    return frame

# show masked message
def masked_msg(frame):
    msg = MASKEDANDLOWTEMP_MSG
    cv2.rectangle(frame, (0, 0), (650, 30), (0,255,0), -1)
    cv2.putText(frame, msg, (300, 21), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)
    return frame

def notpass_msg(frame):
    msg = MASKEDANDHIGHTEMP_MSG
    cv2.rectangle(frame, (0, 0), (650, 30), (0,0,255), -1)
    cv2.putText(frame, msg, (280, 21), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)
    return frame

# show unmasked message
def introduction_unmasked_msg(frame):
    msg = INTRODUCTION_UNMASKEDMSG_MSG
    cv2.rectangle(frame, (0, 60), (650, 90), (0,128,255), -1)
    cv2.putText(frame, msg, (20, 82), cv2.FONT_HERSHEY_TRIPLEX, 0.65, (0, 0, 0), 1, cv2.LINE_AA)
    return frame

def introduction_hightemp_msg(frame, pattern):
    msg = INTRODUCTION_HIGHTEMP_MSG
    if(pattern == 'm' or pattern == 'o'):
        cv2.rectangle(frame, (0, 90), (650, 120), (0,128,255), -1)
        cv2.putText(frame, msg, (70, 112), cv2.FONT_HERSHEY_TRIPLEX, 0.65, (0, 0, 0), 1, cv2.LINE_AA)
    else:   
        cv2.rectangle(frame, (0, 60), (650, 90), (0,128,255), -1)
        cv2.putText(frame, msg, (70, 82), cv2.FONT_HERSHEY_TRIPLEX, 0.65, (0, 0, 0), 1, cv2.LINE_AA)
    return frame

# show nobody message
def nobody_msg(frame):
    msg = NOBODY_MSG
    # rectangle color (B,G,R)
    cv2.rectangle(frame, (0, 0), (650, 30), (0,165,255), -1)
    cv2.putText(frame, msg, (110, 21), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)
    return frame

# show thermal message
def thermal_msg(frame, temp, TEMP_THRESHOLD):
    # show tempreture
    if temp == 0:
        cv2.rectangle(frame, (0, 30), (650, 60), (128,128,128), -1)
        cv2.putText(frame, 'No heat sensor, please re-scan.', (120, 52), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)                
    elif temp > TEMP_THRESHOLD:
        cv2.rectangle(frame, (0, 30), (650, 60), (0,0,255), -1)
        cv2.putText(frame, 'Temperature '+ str(round(temp, 1)) + 'C', (230, 52), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)
    else:
        cv2.rectangle(frame, (0, 30), (650, 60), (255,144,30), -1)
        cv2.putText(frame, 'Temperature '+ str(round(temp, 1)) + 'C', (230, 52), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)

    return frame


def writeTextToImageResult(frame, text1, text2, text3):
    # rectangle color (B,G,R)
    cv2.rectangle(frame, (0, 0), (650, 30), (235,140,52), -1)
    cv2.putText(frame, text1, (25, 21), cv2.FONT_HERSHEY_TRIPLEX, 0.65, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.rectangle(frame, (0, 30), (650, 60), (235,140,52), -1)
    cv2.putText(frame, text2, (25, 52), cv2.FONT_HERSHEY_TRIPLEX, 0.65, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.rectangle(frame, (0, 60), (650, 90), (235,140,52), -1)
    cv2.putText(frame, text3, (25, 82), cv2.FONT_HERSHEY_TRIPLEX, 0.65, (0, 0, 0), 1, cv2.LINE_AA)
    return frame