import cv2
import numpy as np
from  stackImages import stackImages

# frameWidth = 800
# frameHeight = 800
# cap = cv2.VideoCapture(0)
# cap.set(3, frameWidth)
# cap.set(4, frameHeight)

def gstreamer_pipeline(
    capture_width=650,
    capture_height=720,
    display_width=600,
    display_height=600,
    framerate=60,
    flip_method=0,
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
    
cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
path = 'Cropped_dcvuhbpeluxbjbkodmom126.jpg'
while True:
    success, img = cap.read()
    img2 = cv2.imread(path)
    img2 = cv2.resize(img2, (650, 720))
    # cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
    # cv2.setWindowProperty('frame', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    # img = img[200:-1, 200:-1]
    img = cv2.resize(img, (650, 720))
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    StackedImages = stackImages(([img, img2]), 0.6)
    cv2.imshow("PROJECT-C10 GUI CAMERA", StackedImages)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
