"""trt_mtcnn.py

This script demonstrates how to do real-time face detection with
Cython wrapped TensorRT optimized MTCNN engine.
"""

import time
import argparse

import cv2
from utils_mtcnn.camera import add_camera_args, Camera
from utils_mtcnn.display import open_window, set_display, show_fps
from utils_mtcnn.mtcnn import TrtMtcnn


WINDOW_NAME = 'TrtMtcnnDemo'
BBOX_COLOR = (0, 255, 0)  # green


def show_faces(img, boxes, landmarks):
    print("Draw bounding boxes and face landmarks on image.")
    for bb, ll in zip(boxes, landmarks):
        x1, y1, x2, y2 = int(bb[0]), int(bb[1]), int(bb[2]), int(bb[3])
        cv2.rectangle(img, (x1, y1), (x2, y2), BBOX_COLOR, 2)
        for j in range(5):
            cv2.circle(img, (int(ll[j]), int(ll[j+5])), 2, BBOX_COLOR, 2)
    # cv2.imshow(WINDOW_NAME, img)
    # cv2.waitKey(0)

def loop_and_detect(img, mtcnn):
    dets, landmarks = mtcnn.detect(img)
    if(dets.size == 0 and landmarks.size == 0):
        # print('0')
        return 0
    else :
        # img = show_faces(img, dets, landmarks)
        # print('1')
        return 1


def main_detect(image):
    # load model detect face
    mtcnn = TrtMtcnn()
    
    value = loop_and_detect(image, mtcnn)
    # print('value is ',value)
    return value
