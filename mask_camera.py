import cv2
import time
import tensorflow
from PIL import Image, ImageOps
import numpy as np

import busio
import board

import adafruit_amg88xx

#messages
MASKED_MSG = 'PASS'
UNMASKED_MSG = 'Please wear a mask.'
NOBODY_MSG = 'Please set your face on the frame.'

# cariblation temperature
TEMP_CARIBRATION = 7.0
TEMP_THRESHOLD = 36.0

#===================
# class for AMG8833
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
            # print('self.device.temperature ',self.device.temperature)
            # print('self.device.pixels ',self.device.pixels)
            return self.device.pixels, self.device.temperature
    


# overlay frame PNG with alpha and video frame
def overlay(cv_background_image, cv_overlay_image, point):
    overlay_height, overlay_width = cv_overlay_image.shape[:2]
    # เซตในส่วนของรูปภาพที่ได้จากกล้อง
    cv_rgb_bg_image = cv2.cvtColor(cv_background_image, cv2.COLOR_BGR2RGB)
    pil_rgb_bg_image = Image.fromarray(cv_rgb_bg_image)
    pil_rgba_bg_image = pil_rgb_bg_image.convert('RGBA')

    # เซตในส่วนของรูปที่เป็นกรอบ
    cv_rgb_ol_image = cv2.cvtColor(cv_overlay_image, cv2.COLOR_BGRA2RGBA)
    pil_rgb_ol_image = Image.fromarray(cv_rgb_ol_image)
    pil_rgba_ol_image = pil_rgb_ol_image.convert('RGBA')

    pil_rgba_bg_temp = Image.new('RGBA', pil_rgba_bg_image.size,(255, 255, 255, 0))
    pil_rgba_bg_temp.paste(pil_rgba_ol_image, point, pil_rgba_ol_image)
    result_image = \
        Image.alpha_composite(pil_rgba_bg_image, pil_rgba_bg_temp)

    cv_bgr_result_image = cv2.cvtColor(
        np.asarray(result_image), cv2.COLOR_RGBA2BGRA)

    return cv_bgr_result_image

def cv2pil(imgCV):
    imgCV_RGB = imgCV[:, :, ::-1]
    imgPIL = Image.fromarray(imgCV_RGB)
    return imgPIL

# show masked message
def masked_msg(frame):
    msg = MASKED_MSG
    cv2.rectangle(frame, (0, 0), (640, 20), (0,255,0), -1)
    cv2.putText(frame, msg, (100, 18), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)
    return frame

# show unmasked message
def unmasked_msg(frame):
    msg = UNMASKED_MSG
    cv2.rectangle(frame, (0, 0), (640, 20), (0,0,255), -1)
    cv2.putText(frame, msg, (200, 18), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)
    return frame

# show nobody message
def nobody_msg(frame):
    msg = NOBODY_MSG
    # rectangle color (B,G,R)
    cv2.rectangle(frame, (0, 0), (640, 20), (0,165,255), -1)
    cv2.putText(frame, msg, (100, 18), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)
    # reflash temp display area based on gray color
    cv2.rectangle(frame, (0, 21), (640, 45), (0,165,255), -1)
    return frame

def signature_msg(frame):
    cv2.putText(frame, '2015 - 2020 Copyright(c) Forex Robotics Co., Ltd.', (100, 470), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (50, 50, 50), 1, cv2.LINE_AA)
    return frame

# show thermal message
def thermal_msg(frame, pixels):
    if pixels == None:
        # pixels == None means no device avaiable
        temp = 0
    else:
        # find max temprature in 8X8 then add caribiration value
        temp =  max(max(pixels)) + TEMP_CARIBRATION

    # show tempreture
    if temp == 0:
        cv2.rectangle(frame, (0, 21), (640, 45), (128,128,128), -1)
        cv2.putText(frame, 'No thermal sensor available.', (120, 42), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)                
    elif temp > TEMP_THRESHOLD:
        cv2.rectangle(frame, (0, 21), (640, 45), (0,0,255), -1)
        cv2.putText(frame, str(round(temp, 1)) + 'C', (300, 42), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)
    else:
        cv2.rectangle(frame, (0, 21), (640, 45), (255,0,0), -1)
        cv2.putText(frame, str(round(temp, 1)) + 'C', (300, 42), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)

    return frame

def identify_maskman(device_num, delay=1, window_name='mask camera'):
    # set video frame size
    cap = cv2.VideoCapture(device_num)

    # load keras model from Teachable machine
    np.set_printoptions(suppress=True)
    model = tensorflow.keras.models.load_model('./converted_keras/keras_model.h5')
    # print('load model.h5...')
    data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)

    # When can not camera open
    if not cap.isOpened():
        print('can not open camera device: ' + str('device_num'))
        return
    
    #initialize thermal sensor
    sensor = amg8833(0x69)

    # load frame image
    overlay_img = cv2.imread("frame2.png", cv2.IMREAD_UNCHANGED)
    overlay_img = cv2.resize(overlay_img, (640, 480))

    while True:
        # get thermal data
        pixels, unit_temp = sensor.get_tempreture()
        print('thermal data', max(max(pixels)) + TEMP_CARIBRATION)

        # input video frame fro webccam
        ret, frame = cap.read()

        # resize 224 X 224
        size = (224, 224)
        image = ImageOps.fit(cv2pil(frame), size, Image.ANTIALIAS)
        #turn the image into a numpy array
        image_array = np.asarray(image)

        # Normalize the image
        normalized_image_array = (image_array.astype(np.float32) / 127.0) - 1

        # Load the image into the array
        data[0] = normalized_image_array

        # run the inference
        prediction = model.predict(data)
        print('prediction ',prediction)
        msg = ''

        # overlay frame and image
        point = (0,0)
        frame = overlay(frame, overlay_img, point)

        # prediction masked / unmasked 
        if prediction[0][0] > 0.9:
            # เซตค่าตัวหนังสือใส่หน้ากาก
            # show masked message
            frame = masked_msg(frame)
            #show thermal message
            frame = thermal_msg(frame, pixels)
        elif prediction[0][1] > 0.9:
            # เซตค่าตัวหนังสือไม่ใส่หน้ากาก
            # show unmasked message
            frame = unmasked_msg(frame)
            #show thermal message
            frame = thermal_msg(frame, pixels)
        else:
            # nobody
            frame = nobody_msg(frame)
        # show signature message
        frame = signature_msg(frame)
        # show frame
        cv2.imshow(window_name, frame)
        
        # exit when enter [q] key
        key = cv2.waitKey(delay) & 0xFF
        if key == ord('q'):
            break

    # exit process
    cv2.destroyWindow(window_name)

def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
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


if __name__ == '__main__':
    
    # call mask identified program
    identify_maskman(gstreamer_pipeline(flip_method=0))
